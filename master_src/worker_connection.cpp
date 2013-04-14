#include "worker_connection.h"

#include <unistd.h>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "netutils.h"
#include "protocol.h"
#include "visualization.h"

WorkerConnection::WorkerConnection(int fd, Master& master, char* ip_address, int32_t num_updates) :
		ConnectionHandler(fd) {

	this->master = &master;
	this->ip_address = ip_address;
	this->id = 0;
	this->num_updates = num_updates;
	update_count = 0;
	left_neighbour = NULL;
	right_neighbour = NULL;
	next_expected_message = protocol::LISTENING_FOR_NEIGHBOUR_MESSAGE;

}

WorkerConnection::~WorkerConnection() {
	delete[] ip_address;
}

const char* WorkerConnection::get_ip_address() {
	return ip_address;
}

void WorkerConnection::set_id(uint32_t id) {
	this->id = id;
}

Master& WorkerConnection::get_master() {
	return *master;
}

void WorkerConnection::verify_message_expected(unsigned char message_type) {

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Received '%s' message from '%s'(%d) \n", protocol::get_message_type_name(message_type).c_str(),
			get_ip_address(), id);
#endif

	if (message_type != next_expected_message) {
		fprintf(stderr, "[Err] Received '%s' message from '%s'(%d) when expecting '%s'\n",
				protocol::get_message_type_name(message_type).c_str(), get_ip_address(), id,
				protocol::get_message_type_name(next_expected_message).c_str());
		close(fd);
		exit(EXIT_FAILURE);
	}
}

void WorkerConnection::handle_message(unsigned char *message, uint32_t length) {

	verify_message_expected(message[0]);

	switch (message[0]) {

		case protocol::NEIGHBOURS_SET_MESSAGE:
			handle_neighbours_set();
			break;

		case protocol::LISTENING_FOR_NEIGHBOUR_MESSAGE:
			handle_listening_for_neighbour();
			break;

		case protocol::UNIVERSE_PARAMETERS_SET_MESSAGE:
			handle_univ_params_set();
			break;

		case protocol::ROBOTS_SET_MESSAGE:
			handle_robots_set();
			break;

		case protocol::FRAME_FINISHED_MESSAGE:
			handle_frame_finished();
			break;

		case protocol::FRAME_FINISHED_WITH_STATS_MESSAGE:
			handle_frame_with_stats_finished(message);
			break;

		case protocol::FINAL_POSITIONS_MESSAGE:
			handle_final_positions(message);
			break;

		default:
			//If we've reached here, we have closed the socket due to an invalid message
			break;
	}
}

void WorkerConnection::handle_listening_for_neighbour() {

	//Send worker it's right neighbour that it should connect to
	send_message.resize(netutils::IP_ADDRESS_LENGTH + 5);
	send_message.at(0) = protocol::RIGHT_NEIGHBOUR_DISCOVER_MESSAGE;
	netutils::insert_uint32_into_message(netutils::IP_ADDRESS_LENGTH, &send_message[1]);
	memcpy(&send_message[5], right_neighbour->ip_address, netutils::IP_ADDRESS_LENGTH);

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending 'RIGHT_NEIGHBOUR_DISCOVER_MESSAGE' to '%s'(%d)\n", get_ip_address(), id);
#endif
	protocol::send_message(fd, send_message);

	next_expected_message = protocol::NEIGHBOURS_SET_MESSAGE;
}

void WorkerConnection::handle_neighbours_set() {
	//Wait on master to report that neighbours are listening
	master->wait_on_master(id);

	//Notify worker of universe parameters
	send_message.resize(29);
	send_message.at(0) = protocol::SET_UNIVERSE_PARAMETERS_MESSAGE;
	netutils::insert_uint32_into_message(master->get_args().get_world_size(), &send_message[1]);
	netutils::insert_uint32_into_message(master->get_args().get_robot_range(), &send_message[5]);
	netutils::insert_uint32_into_message(master->get_args().get_num_updates(), &send_message[9]);
	netutils::insert_uint32_into_message(master->get_args().get_num_blocks(), &send_message[13]);
	netutils::insert_uint32_into_message(master->get_args().is_visualization_enabled() ? 1 : 0, &send_message[17]);
	netutils::insert_uint32_into_message(master->get_args().get_fov(), &send_message[21]);
	netutils::insert_uint32_into_message(master->get_args().is_direction_inverted() ? 1 : 0, &send_message[25]);
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending 'SET_UNIVERSE_PARAMETERS_MESSAGE' to '%s'(%d)\n", get_ip_address(), id);
#endif
	protocol::send_message(fd, send_message);

	next_expected_message = protocol::UNIVERSE_PARAMETERS_SET_MESSAGE;
}

void WorkerConnection::handle_univ_params_set() {
	//Wait on master to send SET_ROBOTS_MESSAGE
	master->wait_on_master(id);
	next_expected_message = protocol::ROBOTS_SET_MESSAGE;
}

void WorkerConnection::handle_robots_set() {
	//Wait on master to begin simulation
	master->wait_on_master(id);
	send_message = {protocol::START_SIMULATION_MESSAGE};
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending 'START_SIMULATION_MESSAGE' to '%s'(%d)\n", get_ip_address(), id);
#endif
	protocol::send_message(fd, send_message);
	if (num_updates != 0) {
		if (master->get_args().is_visualization_enabled()) {
			next_expected_message = protocol::FRAME_FINISHED_WITH_STATS_MESSAGE;
		} else {
			next_expected_message = protocol::FRAME_FINISHED_MESSAGE;
		}
	} else {
		next_expected_message = protocol::FINAL_POSITIONS_MESSAGE;
	}
}

void WorkerConnection::handle_frame_finished() {
	master->frame_completed(id);
	if (num_updates >= 0 && update_count == num_updates - 1) {
		next_expected_message = protocol::FINAL_POSITIONS_MESSAGE;
	}
	update_count++;
}

void WorkerConnection::handle_frame_with_stats_finished(unsigned char* message) {
	uint32_t num_blocks = netutils::get_uint32_from_message(message + 1);
	for (unsigned int i = 0; i < num_blocks; i++) {
		uint32_t offset = i * 12;
		uint32_t x = netutils::get_uint32_from_message(message + 5 + offset);
		uint32_t y = netutils::get_uint32_from_message(message + 9 + offset);
		uint32_t size = netutils::get_uint32_from_message(message + 13 + offset);
		visualization::set_block_stat(x, y, size);
	}
	master->frame_completed(id);

	if (num_updates >= 0 && update_count == num_updates - 1) {
		next_expected_message = protocol::FINAL_POSITIONS_MESSAGE;
	}
	update_count++;
}

void WorkerConnection::handle_final_positions(unsigned char* message) {
	// Update master's collection of robots
	uint32_t num_robots = netutils::get_uint32_from_message(message + 1);
	for (unsigned int i = 0; i < num_robots; i++) {
		unsigned char* p = message + 5 + (i * Robot::NORMAL_SERIALIZED_LENGTH);
		uint32_t robot_id = Robot::get_id_from_serialized(p);
		Robot *robot = master->get_robot(robot_id);
		robot->update_from_serialized(p, Robot::NORMAL_SERIALIZED_VERSION);
	}
	master->wait_on_master(id);
}

void WorkerConnection::set_left_neighbour(WorkerConnection &left_neighbour) {
	this->left_neighbour = &left_neighbour;
}

void WorkerConnection::set_right_neighbour(WorkerConnection &right_neighbour) {
	this->right_neighbour = &right_neighbour;
}

void WorkerConnection::start() {
	//Wait until master releases the lock so we can start
	master->get_and_release_lock();

	while (true) {
		int result = fill_buffer_and_process();
		if (result <= 0) {
			if (result == 0) {
				fprintf(stderr, "[Err] Worker '%s'(%d) closed the connection\n", get_ip_address(), id);
			} else {
				fprintf(stderr, "[Err] Failed retrieving message from socket\n");
			}
			exit(EXIT_FAILURE);
		}

	}
}

