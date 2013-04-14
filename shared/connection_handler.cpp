#include "connection_handler.h"

#include <netdb.h>
#include <sys/socket.h>
#include <cstring>

ConnectionHandler::ConnectionHandler(int fd) {
	this->fd = fd;
	unprocessed_bytes = 0;
}

ConnectionHandler::~ConnectionHandler() {
}

int ConnectionHandler::fill_buffer_and_process() {

	if (BUFFER_SIZE - unprocessed_bytes == 0) {
		fprintf(stderr, "[Err] ConnectionHandler buffer size is too small for incoming messages\n");
		exit(EXIT_FAILURE);
	}
	int result = recv(fd, recv_buffer + unprocessed_bytes, BUFFER_SIZE - unprocessed_bytes, 0);
	if (result > 0 || unprocessed_bytes > 0) {
		unprocessed_bytes += result;
		process_buffer();
		return 1;
	}
	return result;
}

void ConnectionHandler::process_buffer() {

	// Do we have the payload length? (4 byte header)
	while (unprocessed_bytes >= 4) {
		uint32_t payload_len;
		memcpy(&payload_len, recv_buffer, sizeof(uint32_t));
		payload_len = ntohl(payload_len);

		//Have we received the full payload?
		if (unprocessed_bytes < payload_len + 4) {
			break;
		}

		handle_message(&recv_buffer[4], payload_len);

		//Shuffled remaining data down if applicable
		unprocessed_bytes -= (payload_len + 4);
		if (unprocessed_bytes > 0) {
			memmove(recv_buffer, recv_buffer + 4 + payload_len, unprocessed_bytes);
		}
	}
}

int ConnectionHandler::get_fd() {
	return fd;
}
