#ifndef WORKER_CONNECTION_H_
#define WORKER_CONNECTION_H_

#include "connection_handler.h"

#include <inttypes.h>
#include <pthread.h>
#include <string>

#include "master.h"

//Forward declaration
class Master;

/**
 *
 * Wrapper around a worker socket connection that send/responds to messages. Intended to be ran in its own thread.
 * Communications with the "Master" thread via a direct reference
 *
 */
class WorkerConnection: public ConnectionHandler {

	private:
		Master *master;
		char* ip_address;
		uint32_t id;

		//A send message buffer
		std::vector<unsigned char> send_message;

		//Number of updates
		int32_t num_updates;
		int32_t update_count;

		//Left and right neighbours
		WorkerConnection *left_neighbour;
		WorkerConnection *right_neighbour;

		unsigned char next_expected_message;

		void handle_message(unsigned char *message, uint32_t length);
		void handle_join();
		void handle_listening_for_neighbour();
		void handle_neighbours_set();
		void handle_univ_params_set();
		void handle_robots_set();
		void handle_frame_finished();
		void handle_frame_with_stats_finished(unsigned char* message);
		void handle_final_positions(unsigned char *message);

		void verify_message_expected(unsigned char message_type);

	public:
		WorkerConnection(int fd, Master& master, char* ip_address, int32_t num_updates);
		~WorkerConnection();

		const char* get_ip_address();

		void set_id(uint32_t id);

		Master& get_master();

		//Sets our left & right neighbours respectively
		void set_left_neighbour(WorkerConnection &left_neighbour);
		void set_right_neighbour(WorkerConnection &right_neighbour);

		//Main connection routine
		void start();
};

#endif /* WORKER_CONNECTION_H_ */
