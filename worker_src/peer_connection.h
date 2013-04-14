#ifndef PEER_CONNECTION_H_
#define PEER_CONNECTION_H_

#include <pthread.h>
#include <inttypes.h>
#include <vector>

#include "connection_handler.h"
#include "worker.h"

//Forward declaration
class Worker;

/**
 * Wrapper around a worker to worker socket connection. Intended to be ran in its own thread. Communicates with the
 * "Worker" thread via a direct reference
 *
 */
class PeerConnection: public ConnectionHandler {

	private:
		Worker *worker;
		char* ip_address;
		bool neighboured;
		uint32_t id;
		uint32_t connection_type;

		unsigned char next_expected_message;

		void verify_message_expected(unsigned char message_type);

		void handle_message(unsigned char *message, uint32_t length);
		void handle_neighbour_request();
		void handle_neighbour_request_ack();
		void handle_ghost_strip_message(unsigned char *message);
		void handle_add_robots_message(unsigned char *message);

		void send_ghost_strip();

	public:

		static const uint32_t LEFT_PEER_CONNECTION = 0;
		static const uint32_t RIGHT_PEER_CONNECTION = 1;

		PeerConnection(int fd, Worker& worker, char* ip_address, uint32_t id, unsigned char first_expected_message);
		~PeerConnection();

		const char* get_ip_address();

		//Main connection routine
		void start();
};

#endif /* PEER_CONNECTION_H_ */
