#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "master.h"
#include "netutils.h"
#include "protocol.h"
#include "arguments.h"
#include "visualization.h"

const char* Master::POSITIONS_DUMP_FILE = "robot_positions.txt";

Master::Master(Arguments &args) {

	//Set arguments
	this->args = &args;

	//Load up address structs with getaddrinfo
	struct addrinfo hints, *res;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	if (getaddrinfo(NULL, protocol::SERVER_PORT, &hints, &res) != 0) {
		fprintf(stderr, "[Err] Failed to get address info\n");
		exit(EXIT_FAILURE);
	}

	//Loop through all the results and bind to the first we can
	struct addrinfo *p;
	for (p = res; p != NULL; p = p->ai_next) {

		//Get the socket
		if ((listen_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) < 0) {
			continue;
		}

		// Avoid "Address already in use" error message
		int yes = 1;
		if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) != 0) {
			fprintf(stderr, "[Err] setsockopt failed\n");
			exit(EXIT_FAILURE);
		}

		// Bind the socket
		if (bind(listen_fd, p->ai_addr, p->ai_addrlen) != 0) {
			close(listen_fd);
			continue;
		}

		break;
	}
	if (p == NULL) {
		fprintf(stderr, "[Err] Failed to bind socket\n");
		exit(EXIT_FAILURE);
	}

	//All done with this
	freeaddrinfo(res);

	//Listen on the socket
	if (listen(listen_fd, protocol::SERVER_LISTEN_BACKLOG) != 0) {
		fprintf(stderr, "[Err] Failed to listen on socket\n");
		exit(EXIT_FAILURE);
	}

	//Get our hostname/IP address
	if (gethostname(hostname, HOST_NAME_MAX) != 0) {
		fprintf(stderr, "[Err] Failed to acquire hostname\n");
		exit(EXIT_FAILURE);
	}

	//Initialize
	worker_connections.reserve(args.get_num_workers());
	slowest_connection_count.resize(args.get_num_workers(), 0);
	worker_count = 0;
	worker_connections_working.resize(args.get_num_workers(), true);
	num_worker_connections_working = args.get_num_workers();
	update_count = 0;
	robots = NULL;

	struct timeval start;
	gettimeofday(&start, NULL);
	last_fps_seconds = start.tv_sec + start.tv_usec / 1e6;

	if (pthread_mutex_init(&connections_mutex, NULL) != 0 || pthread_mutex_init(&synchronization, NULL) != 0) {
		fprintf(stderr, "[Err] Failed to initialize mutexes\n");
		exit(EXIT_FAILURE);
	}

	if (pthread_cond_init(&master_done, NULL) != 0 || pthread_cond_init(&all_worker_connections_done, NULL) != 0) {
		fprintf(stderr, "[Err] Failed to initialize conditions\n");
		exit(EXIT_FAILURE);
	}

	//Get synchonization lock
	pthread_mutex_lock(&synchronization);
}

void Master::start() {
	printf("Universe Master '%s'\nWaiting for workers to join (%d)...\n", hostname, args->get_num_workers());

	//Setup visualization if applicable
	if (args->is_visualization_enabled()) {
		visualization::start(args->get_num_blocks(), args->get_population_size());
	}

	struct sockaddr_storage new_addr;
	socklen_t addr_len = sizeof(new_addr);

	//Set the socket in non-blocking mode
	fcntl(listen_fd, F_SETFL, O_NONBLOCK);

	//Startup lobby...accepting connections
	while (worker_count < args->get_num_workers()) {
		int new_fd = accept(listen_fd, (sockaddr *) &new_addr, &addr_len);
		if (new_fd < 0) {
			usleep(100000);
			continue;
		}

		//Set TCP no delay on socket for performance
		if (netutils::disable_nagle_algorithm(new_fd) != 0) {
			fprintf(stderr, "[Err] Failed to set TCP_NODELAY flag on socket\n");
			exit(EXIT_FAILURE);
		}

		pthread_t thread;
		char *ip_address = new char[netutils::IP_ADDRESS_LENGTH];
		netutils::get_ip_textual(&new_addr, ip_address);
		WorkerConnection *wc = new WorkerConnection(new_fd, *this, ip_address, args->get_num_updates());

#ifdef NET_DEBUG
		printf("[NET_DEBUG] New TCP connection with '%s'\n", wc->get_ip_address());
#endif

		if (pthread_create(&thread, NULL, &handle_connection, (void*) wc) != 0) {
			fprintf(stderr, "[Err] Failed to create thread to handle connection\n");
			exit(EXIT_FAILURE);
		}
	}
	//Everyone has connected, stop listening
	close(listen_fd);

	printf("All workers have joined, initializing...\n");

	//Set WorkerConnection neighbours
	for (unsigned int i = 0; i < args->get_num_workers(); i++) {
		WorkerConnection *right;
		if (i < args->get_num_workers() - 1) {
			right = worker_connections.at(i + 1);
		} else {
			right = worker_connections.at(0);
		}
		worker_connections.at(i)->set_right_neighbour(*right);
	}

	//Start workers and wait until workers notify us that all neighbours are set
	start_wait_worker_connections();

	printf("   Peer worker connections established\n");

	//Wait until workers are notified of the universe parameters
	wait_on_worker_connections();

	printf("   Universe parameters set\n");

	//Send each worker its robots & then wait for acks back
	send_robots_to_workers();
	wait_on_worker_connections();

	printf("   Data structures set\nUniverse is ready, press enter to begin simulation...");
	std::cin.ignore();

	simulation_loop();
}

void* Master::handle_connection(void* wc) {
	WorkerConnection *connection = (WorkerConnection *) wc;

#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Thread %lu is handling connection to '%s'\n", pthread_self(), connection->get_ip_address());
#endif

	//We are expecting a JOIN message
	std::vector<unsigned char> message;
	protocol::recieve_message(connection->get_fd(), message);
	if (message.at(0) != protocol::JOIN_MESSAGE) {
		fprintf(stderr, "[Err] Received invalid message when expecting JOIN\n");
		close(connection->get_fd());
		return NULL;
	}
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Received 'JOIN' message from '%s'\n", connection->get_ip_address());
#endif

	//Get this worker's ID
	int id = connection->get_master().worker_joined(*connection);
	connection->set_id(id);

	//Send JOIN_ACK back
	message.resize(9);
	message.at(0) = protocol::JOIN_ACK_MESSAGE;
	netutils::insert_uint32_into_message(id, &message[1]);
	netutils::insert_uint32_into_message(connection->get_master().get_args().get_num_workers(), &message[5]);
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending 'JOIN_ACK' message back to '%s'(%d)\n", connection->get_ip_address(), id);
#endif
	protocol::send_message(connection->get_fd(), message);

	printf("   (%d) '%s' has joined\n", id, connection->get_ip_address());

	//This is now a functioning worker connection, start it
	connection->start();

	close(connection->get_fd());
	return NULL;
}

uint32_t Master::worker_joined(WorkerConnection& worker_connection) {
	pthread_mutex_lock(&connections_mutex);
	worker_connections.push_back(&worker_connection);
	uint32_t id = ++worker_count;
	pthread_mutex_unlock(&connections_mutex);
	return id;
}

void Master::start_wait_worker_connections() {
#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Master (%lu): Finished task, signaling worker connections and sleeping...\n", pthread_self());
#endif
	num_worker_connections_working = args->get_num_workers();
	std::fill(worker_connections_working.begin(), worker_connections_working.end(), true);
	pthread_cond_broadcast(&master_done);
	while (num_worker_connections_working > 0) {
		pthread_cond_wait(&all_worker_connections_done, &synchronization);
	}
#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Master (%lu): Worker connections finished tasks, resuming...\n", pthread_self());
#endif
	pthread_mutex_unlock(&synchronization);
}

void Master::wait_on_worker_connections() {
	pthread_mutex_lock(&synchronization);
	start_wait_worker_connections();
}

void Master::wait_on_master(uint32_t id) {

	pthread_mutex_lock(&synchronization);
	worker_connections_working.at(id - 1) = false;
	num_worker_connections_working--;
	if (num_worker_connections_working == 0) {
#ifdef THREAD_DEBUG
		printf("[THREAD_DEBUG] WorkerConnection (%lu): Finished task. Signaling master and sleeping...\n", pthread_self());
#endif
		pthread_cond_signal(&all_worker_connections_done);
	} else {
#ifdef THREAD_DEBUG
		printf("[THREAD_DEBUG] WorkerConnection (%lu): Finished task, sleeping...\n", pthread_self());
#endif
	}
	while (!worker_connections_working.at(id - 1)) {
		pthread_cond_wait(&master_done, &synchronization);
	}
#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] WorkerConnection (%lu): Master finished task, resuming...\n", pthread_self());
#endif
	pthread_mutex_unlock(&synchronization);
}

void Master::get_and_release_lock() {
	pthread_mutex_lock(&synchronization);
	pthread_mutex_unlock(&synchronization);
}

void Master::send_robots_to_workers() {
	robots = new std::vector<Robot>(args->get_population_size());
	std::vector<Robot*> worker_robots[worker_count];
	uint32_t slice_size = args->get_worker_slice_size();

	//Add each robots to the correct worker collection
	for (unsigned int i = 0; i < robots->size(); i++) {
		for (unsigned int j = 0; j < worker_count; j++) {
			uint32_t x_position = robots->at(i).get_x_position();
			if (x_position >= j * slice_size && x_position < (j + 1) * slice_size) {
				worker_robots[j].push_back(&robots->at(i));
				break;
			}
		}
	}

	//Create & send each message to worker
	std::vector<unsigned char> message;
	for (unsigned int i = 0; i < worker_count; i++) {
		message.resize(5 + (worker_robots[i].size() * Robot::LONG_SERIALIZED_LENGTH));
		message.at(0) = protocol::SET_ROBOTS_MESSAGE;
		netutils::insert_uint32_into_message(worker_robots[i].size(), &message[1]);
		for (unsigned int j = 0; j < worker_robots[i].size(); j++) {
			worker_robots[i].at(j)->serialize_long(&message[5 + (j * Robot::LONG_SERIALIZED_LENGTH)]);
		}
		protocol::send_message(worker_connections.at(i)->get_fd(), message);
	}

}

void Master::simulation_loop() {
	struct timeval now;
	gettimeofday(&now, NULL);
	double start_seconds = now.tv_sec + now.tv_usec / 1e6;

	// Let worker connections run until we reach our frame limit (if applicable)
	wait_on_worker_connections();

	// Get the elapsed time
	gettimeofday(&now, NULL);
	double seconds = now.tv_sec + now.tv_usec / 1e6;
	double elapsed_seconds = seconds - start_seconds;

	dump_robot_positions();
	printf("\nAll done. Elapsed time: %.2f seconds\n", elapsed_seconds);

	// Show worker debug info if applicable
	if (args->is_worker_debug_enabled()) {
		printf("Worker debug info. Slowest for %u updates:\n", args->get_num_updates());
		for (unsigned int i = 0; i < args->get_num_workers(); i++) {
			printf("   %s (%u): %u\n", worker_connections.at(i)->get_ip_address(), i + 1,
					slowest_connection_count.at(i));
		}
	}
}

void Master::frame_completed(uint32_t id) {
	pthread_mutex_lock(&connections_mutex);
	update_count++;
	if (update_count % args->get_num_workers() == 0) {
		uint32_t num_updates = update_count / args->get_num_workers();
		if (num_updates % UPDATE_FRAME_COUNT_PERIOD == 0) {
			struct timeval now;
			gettimeofday(&now, NULL);
			double seconds = now.tv_sec + now.tv_usec / 1e6;
			double interval = seconds - last_fps_seconds;
			printf("[%d] FPS %.1f\r", num_updates, UPDATE_FRAME_COUNT_PERIOD / interval);
			fflush(stdout);
			last_fps_seconds = seconds;
		}

		//Update worker debugging info if enabled
		if (args->is_worker_debug_enabled()) {
			slowest_connection_count.at(id - 1) ++;}
		}

pthread_mutex_unlock	(&connections_mutex);
}

Arguments& Master::get_args() {
	return *args;
}

Robot* Master::get_robot(uint32_t id) {
	return &(robots->at(id - 1));
}

void Master::dump_robot_positions() {
	std::ofstream dump_file(Master::POSITIONS_DUMP_FILE);
	if (dump_file.is_open()) {
		for (unsigned int i = 0; i < robots->size(); i++) {
			dump_file << robots->at(i).to_string_short() << std::endl;
		}
		dump_file.close();
	} else {
		fprintf(stderr, "Failed to open robot positions dump file\n");
		exit(EXIT_FAILURE);
	}
}

//Entry point
int main(int argc, char** argv) {
	//Seed the random generator
	srand48(0);

	//Get and show the desired configuration
	Arguments *args = new Arguments(argc, argv);
	args->print_arguments();

	//Make sure we update Robot's static members with the provided configuration
	Robot::set_world_size(args->get_world_size());

	Master *master = new Master(*args);
	master->start();
}
