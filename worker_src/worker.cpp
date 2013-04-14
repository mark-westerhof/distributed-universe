#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#include "worker.h"
#include "netutils.h"
#include "protocol.h"
#include "robot.h"

Worker::Worker(std::string& master_location) {

	//Load up address structs with getaddrinfo
	struct addrinfo hints, *res;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	if (getaddrinfo(master_location.c_str(), protocol::SERVER_PORT, &hints, &res) != 0) {
		fprintf(stderr, "[Err] Failed to get address info\n");
		exit(EXIT_FAILURE);
	}

	//Loop through all the results and connect to the first we can
	struct addrinfo *p;
	for (p = res; p != NULL; p = p->ai_next) {

		//Get the socket
		if ((master_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
			continue;
		}

		if (connect(master_fd, p->ai_addr, p->ai_addrlen) == -1) {
			close(master_fd);
			continue;
		}

		break;
	}

	if (p == NULL) {
		fprintf(stderr, "[Err] Failed to connect to master\n");
		exit(EXIT_FAILURE);
	}

	master_ip = new char[netutils::IP_ADDRESS_LENGTH];
	netutils::get_ip_textual(p, master_ip);

	//Set TCP no delay on socket for performance
	if (netutils::disable_nagle_algorithm(master_fd) != 0) {
		fprintf(stderr, "[Err] Failed to set TCP_NODELAY flag on socket\n");
		exit(EXIT_FAILURE);
	}

	//All done with this
	freeaddrinfo(res);

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Created TCP connection to master at '%s'\n", master_ip);
#endif

	//Initialize
	num_updates = -1;
	update_count = 0;
	num_blocks = 0;
	block_size = 0;
	id = 0;
	num_workers = 0;
	listening = false;
	left_neighbour = NULL;
	right_neighbour = NULL;
	num_peer_connections_working = 2;
	peer_connections_working[0] = true;
	peer_connections_working[1] = true;
	map = NULL;
	visualization_enabled = false;

	if (pthread_mutex_init(&listening_mutex, NULL) != 0 || pthread_mutex_init(&left_neighbour_mutex, NULL) != 0
			|| pthread_mutex_init(&right_neighbour_mutex, NULL) != 0
			|| pthread_mutex_init(&synchronization, NULL) != 0) {
		fprintf(stderr, "[Err] Failed to initialize mutexes\n");
		exit(EXIT_FAILURE);
	}
	if (pthread_cond_init(&listening_for_neighbour, NULL) != 0 || pthread_cond_init(&worker_done, NULL) != 0
			|| pthread_cond_init(&both_peer_connections_done, NULL) != 0) {
		fprintf(stderr, "[Err] Failed to initialize condition\n");
		exit(EXIT_FAILURE);
	}

	//Get synchonization lock
	pthread_mutex_lock(&synchronization);
}

void Worker::send_message_to_master(std::vector<unsigned char> &message) {
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending '%s' message to master\n", protocol::get_message_type_name(message.at(0)).c_str());
#endif
	protocol::send_message(master_fd, message);
}

void Worker::recieve_message_from_master(std::vector<unsigned char> &message, unsigned char expected_message_type) {
	protocol::recieve_message(master_fd, message);
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Received '%s' message from master\n", protocol::get_message_type_name(message.at(0)).c_str());
#endif

	if (message.at(0) != expected_message_type) {
		fprintf(stderr, "[Err] Received '%s' message from master when expecting '%s'\n",
				protocol::get_message_type_name(message.at(0)).c_str(),
				protocol::get_message_type_name(expected_message_type).c_str());
		close(master_fd);
		exit(EXIT_FAILURE);
	}
}

void Worker::join() {

	//Send JOIN message to master
	std::vector<unsigned char> message = { protocol::JOIN_MESSAGE };
	send_message_to_master(message);

	//Receive JOIN ACK back
	recieve_message_from_master(message, protocol::JOIN_ACK_MESSAGE);
	id = netutils::get_uint32_from_message(&message[1]);
	num_workers = netutils::get_uint32_from_message(&message[5]);

	printf("Connected to master as worker %d of %d\n", id, num_workers);

	//Set up listening socket for left neighbour
	pthread_t thread;
	if (pthread_create(&thread, NULL, &listen_for_neighbour, (void *) this) != 0) {
		fprintf(stderr, "[Err] Failed to create thread to listen for neighbour\n");
		exit(EXIT_FAILURE);
	}

	//Wait until we are listening for our left neighbour
	pthread_mutex_lock(&listening_mutex);
	while (!listening) {
		pthread_cond_wait(&listening_for_neighbour, &listening_mutex);
	}
	pthread_mutex_unlock(&listening_mutex);

	//Notify master that we are now listening for our left neighbour
	message = {protocol::LISTENING_FOR_NEIGHBOUR_MESSAGE};
	send_message_to_master(message);

	//Discover and connect to our right neighbour
	recieve_message_from_master(message, protocol::RIGHT_NEIGHBOUR_DISCOVER_MESSAGE);
	uint32_t ip_addr_len = netutils::get_uint32_from_message(&message[1]);
	char *right_neighbour_ip = new char[ip_addr_len];
	memcpy(right_neighbour_ip, &message[5], ip_addr_len);
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Our right neighbour can be found at '%s'\n", right_neighbour_ip);
#endif
	connect_to_neighbour(right_neighbour_ip);

	//Allow peer connections to start and wait until they are set
	start_wait_peer_connections();

	printf("Peer worker connections established\n");

	//Notify master that peer connections are set
	message = {protocol::NEIGHBOURS_SET_MESSAGE};
	send_message_to_master(message);

	//Receive and set universe parameters & create data structure
	recieve_message_from_master(message, protocol::SET_UNIVERSE_PARAMETERS_MESSAGE);
	Robot::set_world_size(netutils::get_uint32_from_message(&message[1]));
	Robot::range = (netutils::get_uint32_from_message(&message[5]));
	num_updates = netutils::get_uint32_from_message(&message[9]);
	num_blocks = netutils::get_uint32_from_message(&message[13]);
	visualization_enabled = netutils::get_uint32_from_message(&message[17]) == 1 ? true : false;
	Robot::set_fov(netutils::get_uint32_from_message(&message[21]));
	Robot::invert_direction = netutils::get_uint32_from_message(&message[25]) == 1 ? true : false;
	block_size = Robot::get_world_size() / num_blocks;

	uint32_t blocks_per_slice = num_blocks / num_workers;
	uint32_t leftmost_x_index = blocks_per_slice * (id - 1);
	uint32_t rightmost_x_index = (blocks_per_slice * id) - 1;
	map = new RobotMap(num_blocks, leftmost_x_index, rightmost_x_index);

	//Notify master that parameters are set
	message = {protocol::UNIVERSE_PARAMETERS_SET_MESSAGE};
	send_message_to_master(message);

	//Receive our robots
	recieve_message_from_master(message, protocol::SET_ROBOTS_MESSAGE);
	uint32_t num_robots = netutils::get_uint32_from_message(&message[1]);
	for (unsigned int i = 0; i < num_robots; i++) {
		Robot *robot = new Robot(&message[5 + (i * Robot::LONG_SERIALIZED_LENGTH)], Robot::LONG_SERIALIZED_VERSION);
		map->add_robot(*robot);
	}

	printf("Created and populated data structures\n");

	//Notify master that robots have been added
	message = {protocol::ROBOTS_SET_MESSAGE};
	send_message_to_master(message);

	//Wait for master to tell us to start the simulation
	printf("Waiting for master to initiate simulation...\n");
	recieve_message_from_master(message, protocol::START_SIMULATION_MESSAGE);
	printf("Running\n");
	simulation_loop();
}

void Worker::simulation_loop() {

	unsigned char frame_finished_messaged[5];
	netutils::insert_uint32_into_message(1, frame_finished_messaged);
	frame_finished_messaged[4] = protocol::FRAME_FINISHED_MESSAGE;

	bool running = num_updates != 0;
	while (running) {
		if (num_updates > 0 && update_count > num_updates) {
			break;
		}

		map->clear_ghost_strips();
		map->update_robot_positions_and_reset_sensors();

		// Wait while peer connections do:
		//   (1) Send ghost strips
		//   (2) Receive ghost strips
		//   (3) Send robots (transfers)
		//   (4) Receive robots
		wait_on_peer_connections();

		map->update_robot_sensors();
		map->set_robot_speeds_and_directions();

		if (num_updates < 0 || update_count <= num_updates - 1) {
			//Send frame completed
			if (visualization_enabled) {
#ifdef NET_DEBUG
				printf("[NET_DEBUG] Sending 'FRAME_FINISHED_WITH_STATS_MESSAGE' message to master\n");
#endif
				map->send_frame_stats_message(master_fd);
			} else {
#ifdef NET_DEBUG
				printf("[NET_DEBUG] Sending 'FRAME_FINISHED_MESSAGE' message to master\n");
#endif
				protocol::send_message(master_fd, frame_finished_messaged, 5);
			}
		}
		update_count++;
	}
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending 'FINAL_POSITIONS_MESSAGE' message to master\n");
#endif
	map->send_final_positions_message(master_fd);
	printf("Done simulation\n");
	exit(EXIT_SUCCESS);
}

void* Worker::listen_for_neighbour(void* arg) {
	Worker *worker = (Worker*) arg;

#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Thread %lu is listening for left neighbour\n", pthread_self());
#endif

	int listen_fd;

	struct addrinfo hints, *res;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	//Figure our which port we should be listening at for our neighbour according to our id
	int listen_port = protocol::BASE_NEIGHBOUR_PORT + worker->id;

	if (getaddrinfo(NULL, std::to_string(listen_port).c_str(), &hints, &res) != 0) {
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

	struct sockaddr_storage new_addr;
	socklen_t addr_len = sizeof(new_addr);

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Listening for our left neighbour at port %d\n", listen_port);
#endif

	//We are now listening for our neighbour, notify main thread
	pthread_mutex_lock(&(worker->listening_mutex));
	worker->listening = true;
	pthread_cond_signal(&(worker->listening_for_neighbour));
	pthread_mutex_unlock(&(worker->listening_mutex));

	//What's the ID of our left neighbour?
	int left_neighbour_id;
	if (worker->id == 1) {
		left_neighbour_id = worker->num_workers;
	} else {
		left_neighbour_id = worker->id - 1;
	}

	//Accepting connections...
	while (worker->left_neighbour == NULL) {
		int new_fd = accept(listen_fd, (sockaddr *) &new_addr, &addr_len);
		if (new_fd < 0) {
			continue;
		}

		//Set TCP no delay on socket for performance
		if (netutils::disable_nagle_algorithm(new_fd) != 0) {
			fprintf(stderr, "[Err] Failed to set TCP_NODELAY flag on socket\n");
			exit(EXIT_FAILURE);
		}

		if (netutils::set_bdp(new_fd) != 0) {
			fprintf(stderr, "[Err] Failed to set BDP size on socket\n");
			exit(EXIT_FAILURE);
		}

		pthread_t thread;
		char* ip_address = new char[netutils::IP_ADDRESS_LENGTH];
		netutils::get_ip_textual(&new_addr, ip_address);
		PeerConnection *pc = new PeerConnection(new_fd, *worker, ip_address, left_neighbour_id,
				protocol::NEIGHBOUR_REQUEST_MESSAGE);

		if (pthread_create(&thread, NULL, &handle_peer_connection, (void*) pc) != 0) {
			fprintf(stderr, "[Err] Failed to create thread to handle peer connection\n");
			exit(EXIT_FAILURE);
		}
	}
	close(listen_fd);

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Stopped listening for our left neighbour\n");
#endif

	return NULL;
}

void Worker::connect_to_neighbour(char* ip_address) {
	int right_fd;

	//Load up address structs with getaddrinfo
	struct addrinfo hints, *res;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	//What's the ID/listen port of our right neighbour?
	int right_neighbour_id;
	if (id == num_workers) {
		right_neighbour_id = 1;
	} else {
		right_neighbour_id = id + 1;
	}
	std::string right_listen_port = std::to_string(protocol::BASE_NEIGHBOUR_PORT + right_neighbour_id);

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Connecting to our right neighbour '%s'(%d) at port %s\n", ip_address, right_neighbour_id,
			right_listen_port.c_str());
#endif

	if (getaddrinfo(ip_address, right_listen_port.c_str(), &hints, &res) != 0) {
		fprintf(stderr, "[Err] Failed to get address info\n");
		exit(EXIT_FAILURE);
	}

	//Loop through all the results and connect to the first we can
	struct addrinfo *p;
	for (p = res; p != NULL; p = p->ai_next) {

		//Get the socket
		if ((right_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
			continue;
		}

		if (connect(right_fd, p->ai_addr, p->ai_addrlen) == -1) {
			close(right_fd);
			continue;
		}

		break;
	}

	if (p == NULL) {
		fprintf(stderr, "[Err] Failed to connect to right neighbour\n");
		exit(EXIT_FAILURE);
	}

	//Set TCP no delay on socket for performance
	if (netutils::disable_nagle_algorithm(right_fd) != 0) {
		fprintf(stderr, "[Err] Failed to set TCP_NODELAY flag on socket\n");
		exit(EXIT_FAILURE);
	}

	if (netutils::set_bdp(right_fd) != 0) {
		fprintf(stderr, "[Err] Failed to set BDP size on socket\n");
		exit(EXIT_FAILURE);
	}

	//Send NEIGHBOUR_REQUEST message to start
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending 'NEIGHBOUR_REQUEST' message to '%s'(%d)\n", ip_address, right_neighbour_id);
#endif
	std::vector<unsigned char> request_msg = { protocol::NEIGHBOUR_REQUEST_MESSAGE };
	protocol::send_message(right_fd, request_msg);

	pthread_t thread;
	PeerConnection *pc = new PeerConnection(right_fd, *this, ip_address, right_neighbour_id,
			protocol::NEIGHBOUR_REQUEST_ACK_MESSAGE);

	if (pthread_create(&thread, NULL, &handle_peer_connection, (void*) pc) != 0) {
		fprintf(stderr, "[Err] Failed to create thread to handle peer connection\n");
		exit(EXIT_FAILURE);
	}

	//All done with this
	freeaddrinfo(res);
}

void* Worker::handle_peer_connection(void* pc) {
	PeerConnection *connection = (PeerConnection *) pc;

#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Thread %lu is handling peer connection to '%s'\n", pthread_self(), connection->get_ip_address());
#endif

	connection->start();

	close(connection->get_fd());
	return NULL;
}

int Worker::set_left_neighbour(PeerConnection& left_neighbour) {
	int return_value = 0;
	pthread_mutex_lock(&left_neighbour_mutex);
	if (this->left_neighbour == NULL) {
		this->left_neighbour = &left_neighbour;
	} else {
		return_value = -1;
	}
	pthread_mutex_unlock(&left_neighbour_mutex);
	return return_value;
}

int Worker::set_right_neighbour(PeerConnection& right_neighbour) {
	int return_value = 0;
	pthread_mutex_lock(&right_neighbour_mutex);
	if (this->right_neighbour == NULL) {
		this->right_neighbour = &right_neighbour;
	} else {
		return_value = -1;
	}
	pthread_mutex_unlock(&right_neighbour_mutex);
	return return_value;
}

void Worker::start_wait_peer_connections() {
#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Worker (%lu): Finished task, signaling peer connections and sleeping...\n", pthread_self());
#endif
	peer_connections_working[0] = true;
	peer_connections_working[1] = true;
	num_peer_connections_working = 2;
	pthread_cond_broadcast(&worker_done);
	while (num_peer_connections_working > 0) {
		pthread_cond_wait(&both_peer_connections_done, &synchronization);
	}
#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] Worker (%lu): Peer connections finished tasks, resuming...\n", pthread_self());
#endif
	pthread_mutex_unlock(&synchronization);
}

void Worker::wait_on_peer_connections() {
	pthread_mutex_lock(&synchronization);
	start_wait_peer_connections();
}

void Worker::wait_on_worker(uint32_t connection_type) {

	pthread_mutex_lock(&synchronization);
	peer_connections_working[connection_type] = false;
	num_peer_connections_working--;
	if (num_peer_connections_working == 0) {
#ifdef THREAD_DEBUG
		printf("[THREAD_DEBUG] PeerConnection (%lu): Finished task. Signaling worker and sleeping...\n", pthread_self());
#endif
		pthread_cond_signal(&both_peer_connections_done);
	} else {
#ifdef THREAD_DEBUG
		printf("[THREAD_DEBUG] PeerConnection (%lu): Finished task, sleeping...\n", pthread_self());
#endif
	}
	while (!peer_connections_working[connection_type]) {
		pthread_cond_wait(&worker_done, &synchronization);
	}
#ifdef THREAD_DEBUG
	printf("[THREAD_DEBUG] WorkerConnection (%lu): Worker finished task, resuming...\n", pthread_self());
#endif
	pthread_mutex_unlock(&synchronization);
}

void Worker::get_and_release_lock() {
	pthread_mutex_lock(&synchronization);
	pthread_mutex_unlock(&synchronization);
}

uint32_t Worker::get_num_blocks() {
	return num_blocks;
}

RobotMap& Worker::get_map() {
	return *map;
}

//Entry point
int main(int argc, char** argv) {
	if (argc < 2) {
		fprintf(stderr, "Master hostname/IP address not supplied\n");
		printf("\nUsage: %s hostname | IP \n", argv[0]);
		exit(1);
	}

	std::string master_location(argv[1]);
	Worker *worker = new Worker(master_location);
	worker->join();
}
