#include <sys/socket.h>
#include <netdb.h>
#include <inttypes.h>
#include <cstdlib>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "protocol.h"
#include "netutils.h"

std::string protocol::get_message_type_name(const unsigned char message_type) {
	std::string message_name;

	switch (message_type) {

		case protocol::JOIN_MESSAGE:
			message_name = "JOIN";
			break;

		case protocol::JOIN_ACK_MESSAGE:
			message_name = "JOIN_ACK";
			break;

		case protocol::LISTENING_FOR_NEIGHBOUR_MESSAGE:
			message_name = "LISTENING_FOR_NEIGHBOUR";
			break;

		case protocol::RIGHT_NEIGHBOUR_DISCOVER_MESSAGE:
			message_name = "RIGHT_NEIGHBOUR_DISCOVER";
			break;

		case protocol::NEIGHBOURS_SET_MESSAGE:
			message_name = "NEIGHBOURS_SET";
			break;

		case protocol::NEIGHBOUR_REQUEST_MESSAGE:
			message_name = "NEIGHBOUR_REQUEST";
			break;

		case protocol::NEIGHBOUR_REQUEST_ACK_MESSAGE:
			message_name = "NEIGHBOUR_REQUEST_ACK";
			break;

		case protocol::SET_UNIVERSE_PARAMETERS_MESSAGE:
			message_name = "SET_UNIVERSE_PARAMETERS_MESSAGE";
			break;

		case protocol::UNIVERSE_PARAMETERS_SET_MESSAGE:
			message_name = "UNIVERSE_PARAMETERS_SET_MESSAGE";
			break;

		case protocol::SET_ROBOTS_MESSAGE:
			message_name = "SET_ROBOTS_MESSAGE";
			break;

		case protocol::ROBOTS_SET_MESSAGE:
			message_name = "ROBOTS_SET_MESSAGE";
			break;

		case protocol::START_SIMULATION_MESSAGE:
			message_name = "START_SIMULATION_MESSAGE";
			break;

		case protocol::GHOST_STRIP_MESSAGE:
			message_name = "GHOST_STRIP_MESSAGE";
			break;

		case protocol::ADD_ROBOTS_MESSAGE:
			message_name = "ADD_ROBOTS_MESSAGE";
			break;

		case protocol::FRAME_FINISHED_MESSAGE:
			message_name = "FRAME_FINISHED_MESSAGE";
			break;

		case protocol::FRAME_FINISHED_WITH_STATS_MESSAGE:
			message_name = "FRAME_FINISHED_WITH_STATS_MESSAGE";
			break;

		case protocol::FINAL_POSITIONS_MESSAGE:
			message_name = "FINAL_POSITIONS_MESSAGE";
			break;

		default:
			message_name = "UNKNOWN";
			break;

	}
	return message_name;
}

void protocol::recieve_message(int socket, std::vector<unsigned char>& message) {

	//Need to union to abide by strict aliasing rules
	union {
			unsigned char raw[4];
			uint32_t value;
	} header;

	//Get message length header
	int res;
	if ((res = netutils::recvall(socket, header.raw, sizeof(header.raw))) <= 0) {
		if (res == 0) {
			fprintf(stderr, "[Err] Socket closed\n");
		} else {
			fprintf(stderr, "[Err] Failed to receive message header\n");
		}
		exit(EXIT_FAILURE);
	}
	header.value = ntohl(header.value);
	if (header.value <= 0) {
		fprintf(stderr, "[Err] Retrieved message header with invalid length\n");
		exit(EXIT_FAILURE);
	}

	//Get the remainder of the message (size of length header) in host byte order
	message.resize(header.value);
	if ((res = netutils::recvall(socket, &message[0], header.value)) <= 0) {
		if (res == 0) {
			fprintf(stderr, "[Err] Socket closed\n");
		} else {
			fprintf(stderr, "[Err] Failed to receive message header\n");
		}
		exit(EXIT_FAILURE);
	}
}

void protocol::send_message(int socket, const std::vector<unsigned char>& message) {

	//Create new message of length 'len' plus header size
	unsigned char *full_message = new unsigned char[message.size() + 4];

	//Added the length header in network byte order then append the message
	netutils::insert_uint32_into_message(message.size(), full_message);
	memcpy(full_message + 4, &message[0], message.size());

	if (netutils::sendall(socket, full_message, message.size() + 4) != 0) {
		fprintf(stderr, "[Err] Failed to send message\n");
		exit(EXIT_FAILURE);
	}
	delete[] full_message;
}

void protocol::send_message(int socket, unsigned char* message, size_t len) {

	if (netutils::sendall(socket, message, len) != 0) {
		fprintf(stderr, "[Err] Failed to send message\n");
		exit(EXIT_FAILURE);
	}
}

