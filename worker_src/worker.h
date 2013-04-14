#ifndef WORKER_H_
#define WORKER_H_

#include <inttypes.h>
#include <pthread.h>

#include "peer_connection.h"

#include "robot_map.h"

//Forward declaration
class PeerConnection;

class Worker {

	private:

		//Socket connection to master
		int master_fd;

		//Master IP address
		char* master_ip;

		//Our worker id out of
		uint32_t id;
		uint32_t num_workers;

		// Our data structures for robots
		RobotMap *map;

		//Mutexes, conditions, and variables for synchronization of work/wait between worker and peer connections
		pthread_mutex_t synchronization;
		pthread_cond_t worker_done;
		pthread_cond_t both_peer_connections_done;
		unsigned int num_peer_connections_working;
		bool peer_connections_working[2];

		//Number of updates to perform (-1: No limit)
		int32_t num_updates;
		int32_t update_count;

		//Number of blocks in the grid (NxN) & the size of a block
		uint32_t num_blocks;
		uint32_t block_size;

		//Is visualization enabled?
		bool visualization_enabled;

		//Are we currently listening for our left neighbour?
		bool listening;
		pthread_mutex_t listening_mutex;
		pthread_cond_t listening_for_neighbour;

		//Peer connections
		PeerConnection *left_neighbour;
		PeerConnection *right_neighbour;
		pthread_mutex_t left_neighbour_mutex;
		pthread_mutex_t right_neighbour_mutex;

		// Sends a message to master
		void send_message_to_master(std::vector<unsigned char> &message);

		// Receives a message to master after validating
		void recieve_message_from_master(std::vector<unsigned char> &message, unsigned char expected_message_type);

		//Thread routine for listening for right neighbour connection
		static void* listen_for_neighbour(void* worker);

		//Thread routine for handling peer connection
		static void* handle_peer_connection(void* pc);

		//Signals the peer connections to do work while we wait (this is called only the first time)
		void start_wait_peer_connections();

		//Signals the peer connections to do work while we wait
		void wait_on_peer_connections();

		//Connects to our right neighbour at the specified ip
		void connect_to_neighbour(char* ip_address);

		// Runs the main simulation loop
		void simulation_loop();

	public:

		Worker(std::string& master_location);

		//Sets worker's left & righ neighbours respectively. Returns 0: success, -1: already set
		int set_left_neighbour(PeerConnection& left_neighbour);
		int set_right_neighbour(PeerConnection& right_neighbour);

		//Initiates the "join" procedure to master. This puts the worker to work indefinitely or until the update limit
		//is reached (if applicable)
		void join();

		// Signals the worker thread to do work while calling routine (peer connection) waits
		void wait_on_worker(uint32_t connection_type);

		// Quick lock & release for synchronization b/w worker and peer connections. Peer connections should be created
		// with worker already having ownership of the lock
		void get_and_release_lock();

		uint32_t get_num_blocks();

		RobotMap& get_map();
};

#endif /* WORKER_H_ */
