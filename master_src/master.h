#ifndef MASTER_H_
#define MASTER_H_

#include <limits.h>
#include <pthread.h>
#include <inttypes.h>
#include <vector>

#include "worker_connection.h"
#include "arguments.h"
#include "robot.h"

#ifndef HOST_NAME_MAX
#define HOST_NAME_MAX 256
#endif

//Forward declaration
class WorkerConnection;

/**
 *
 * The center most component of the "Master" node. Runs the main thread
 *
 */
class Master {

	private:

		static const int UPDATE_FRAME_COUNT_PERIOD = 10;

		static const char* POSITIONS_DUMP_FILE;

		//The listen socket for incoming connections
		int listen_fd;

		//Configuration arguments
		Arguments *args;

		//Worker connections
		std::vector<WorkerConnection*> worker_connections;
		std::vector<uint32_t> slowest_connection_count;
		uint32_t worker_count;
		pthread_mutex_t connections_mutex;

		//Mutexes, conditions, and variables for synchronization of work/wait between master and worker connections
		pthread_mutex_t synchronization;
		pthread_cond_t master_done;
		pthread_cond_t all_worker_connections_done;
		unsigned int num_worker_connections_working;
		std::vector<bool> worker_connections_working;

		//Amount of updates completed & the time of the last update period
		uint32_t update_count;
		double last_fps_seconds;

		//Contains all robots (created during initialization, updated at the end)
		std::vector<Robot> *robots;

		//Our hostname/IP in readable form
		char hostname[HOST_NAME_MAX];

		//Thread routine for handling worker connections
		static void* handle_connection(void* wc);

		// Adds the worker connects, increments the count, and returns connection number for calling routine
		uint32_t worker_joined(WorkerConnection& worker_connection);

		// Signals the peer connections to do work while we wait (this is called only the first time)
		void start_wait_worker_connections();

		// Signals the worker connections to do work while we wait
		void wait_on_worker_connections();

		// Sends each worker the appropriate collection of robots
		void send_robots_to_workers();

		// Runs the main simulation loop
		void simulation_loop();

		// Dumps final robot positions to a text file
		void dump_robot_positions();

	public:
		Master(Arguments &args);

		Arguments& get_args();

		//Starts the main master routine
		void start();

		// Signals the master thread to do work while calling routine (worker connection) waits
		void wait_on_master(uint32_t id);

		// Quick lock & release for synchonization b/w master and worker connections. Worker connections should be
		// created with master already having ownership of the lock
		void get_and_release_lock();

		// Called from worker connection thread when a frame has finished
		void frame_completed(uint32_t id);

		// Gets the robot with the specific id (for updating once all frames are completed)
		Robot* get_robot(uint32_t id);

};

#endif /* MASTER_H_ */
