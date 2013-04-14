#include "peer_connection.h"

#include <unistd.h>

#include "protocol.h"
#include "netutils.h"

PeerConnection::PeerConnection(int fd, Worker& worker, char* ip_address, uint32_t id,
		unsigned char first_expected_message) :
		ConnectionHandler(fd) {

	this->worker = &worker;
	this->ip_address = ip_address;
	this->id = id;
	connection_type = 0;
	neighboured = false;
	next_expected_message = first_expected_message;
}

PeerConnection::~PeerConnection() {
	delete[] ip_address;
}

const char* PeerConnection::get_ip_address() {
	return ip_address;
}

void PeerConnection::verify_message_expected(unsigned char message_type) {

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Received '%s' message from '%s'(%d) \n", protocol::get_message_type_name(message_type).c_str(),
			get_ip_address(), id);
#endif

	if (message_type != next_expected_message) {
		fprintf(stderr, "[Err] Received '%s' message from '%s'(%d) when expecting '%s'\n",
				protocol::get_message_type_name(message_type).c_str(), get_ip_address(), id,
				protocol::get_message_type_name(next_expected_message).c_str());
		//Fail loudly if functioning neighbour
		if (neighboured) {
			close(fd);
			exit (EXIT_FAILURE);
		} else {
			close(fd);
		}
	}
}

void PeerConnection::handle_message(unsigned char *message, uint32_t length) {

	verify_message_expected(message[0]);

	switch (message[0]) {

		case protocol::NEIGHBOUR_REQUEST_MESSAGE:
			handle_neighbour_request();
			break;

		case protocol::NEIGHBOUR_REQUEST_ACK_MESSAGE:
			handle_neighbour_request_ack();
			break;

		case protocol::GHOST_STRIP_MESSAGE:
			handle_ghost_strip_message(message);
			break;

		case protocol::ADD_ROBOTS_MESSAGE:
			handle_add_robots_message(message);
			break;

		default:
			//If we've reached here, we have closed the socket due to an invalid message
			break;
	}
}

void PeerConnection::handle_neighbour_request() {

	//Attempt to set worker's left neighbour
	if (worker->set_left_neighbour(*this) != 0) {
		fprintf(stderr, "[Err] Left neighbour already set\n");
		exit (EXIT_FAILURE);
	}
	connection_type = PeerConnection::LEFT_PEER_CONNECTION;

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Connected to left neighbour '%s'(%d). Sending NEIGHBOUR_REQUEST_ACK\n", get_ip_address(), id);
#endif

	unsigned char send_message[5];
	netutils::insert_uint32_into_message(1, send_message);
	send_message[4] = protocol::NEIGHBOUR_REQUEST_ACK_MESSAGE;

	protocol::send_message(fd, send_message, 5);
	neighboured = true;

	//Notify worker that peer connection is set and wait until simulation is running
	worker->wait_on_worker(connection_type);

	send_ghost_strip();
	next_expected_message = protocol::GHOST_STRIP_MESSAGE;
}

void PeerConnection::handle_neighbour_request_ack() {

	//Attempt to set worker's right neighbour
	if (worker->set_right_neighbour(*this) != 0) {
		fprintf(stderr, "[Err] Right neighbour already set\n");
		exit (EXIT_FAILURE);
	}
	connection_type = PeerConnection::RIGHT_PEER_CONNECTION;
	neighboured = true;

#ifdef NET_DEBUG
	printf("[NET_DEBUG] Connected to right neighbour '%s'(%d)\n", get_ip_address(), id);
#endif

	//Notify worker that peer connection is set and wait until simulation is running
	worker->wait_on_worker(connection_type);

	send_ghost_strip();
	next_expected_message = protocol::GHOST_STRIP_MESSAGE;
}

void PeerConnection::handle_ghost_strip_message(unsigned char *message) {
	uint64_t message_index = 1;
	for (unsigned int i = 0; i < worker->get_num_blocks(); i++) {
		uint32_t x_coordinate = netutils::get_uint32_from_message(message + message_index);
		uint32_t y_coordinate = netutils::get_uint32_from_message(message + message_index + 4);
		MapCoordinate coordinate = MapCoordinate(x_coordinate, y_coordinate);
		uint32_t num_robots = netutils::get_uint32_from_message(message + message_index + 8);
		message_index += 12;

		for (unsigned int j = 0; j < num_robots; j++) {
			Robot *robot = new Robot(message + message_index, Robot::GHOST_SERIALIZED_VERSION);
			if (connection_type == PeerConnection::LEFT_PEER_CONNECTION) {
				worker->get_map().add_left_ghost_strip_robot(*robot, coordinate);
			} else {
				worker->get_map().add_right_ghost_strip_robot(*robot, coordinate);
			}
			message_index += Robot::GHOST_SERIALIZED_LENGTH;
		}
	}
#ifdef NET_DEBUG
	uint32_t count = 0;
#endif
	if (connection_type == PeerConnection::LEFT_PEER_CONNECTION) {
#ifdef NET_DEBUG
		count = worker->get_map().send_left_moved_robots(fd);
#else
		worker->get_map().send_left_moved_robots(fd);
#endif
	} else {
#ifdef NET_DEBUG
		count = worker->get_map().send_right_moved_robots(fd);
#else
		worker->get_map().send_right_moved_robots(fd);
#endif
	}
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending ADD_ROBOTS_MESSAGE to '%s'(%d) of count %u\n", get_ip_address(), id, count);
#endif

	next_expected_message = protocol::ADD_ROBOTS_MESSAGE;
}

void PeerConnection::handle_add_robots_message(unsigned char *message) {
	uint32_t num_robots = netutils::get_uint32_from_message(message + 1);
	for (unsigned int i = 0; i < num_robots; i++) {
		Robot *robot = new Robot(message + 5 + (i * Robot::LONG_SERIALIZED_LENGTH), Robot::LONG_SERIALIZED_VERSION);
		worker->get_map().add_robot(*robot);
	}

	//Wait for next frame
	worker->wait_on_worker(connection_type);

	send_ghost_strip();
	next_expected_message = protocol::GHOST_STRIP_MESSAGE;
}

void PeerConnection::send_ghost_strip() {
#ifdef NET_DEBUG
	uint32_t count = 0;
#endif
	if (connection_type == PeerConnection::LEFT_PEER_CONNECTION) {
#ifdef NET_DEBUG
		count = worker->get_map().send_left_ghost_strip_message(fd);
#else
		worker->get_map().send_left_ghost_strip_message(fd);
#endif
	} else {
#ifdef NET_DEBUG
		count = worker->get_map().send_right_ghost_strip_message(fd);
#else
		worker->get_map().send_right_ghost_strip_message(fd);
#endif
	}
#ifdef NET_DEBUG
	printf("[NET_DEBUG] Sending GHOST_STRIP_MESSAGE to '%s'(%d) of count %u\n", get_ip_address(), id, count);
#endif

}

void PeerConnection::start() {

	//Wait until worker releases the lock so we can start
	this->worker->get_and_release_lock();

	while (true) {
		int result = fill_buffer_and_process();
		if (result <= 0) {
			//Only fail loudly if this is a functioning neighbour
			if (neighboured) {
				if (result == 0) {
					fprintf(stderr, "[Err] Neighbour '%s'(%d) closed the connection\n", get_ip_address(), id);
				} else {
					fprintf(stderr, "[Err] Failed retrieving message from socket\n");
				}
				exit (EXIT_FAILURE);
			} else {
				break;
			}

		}
	}
}
