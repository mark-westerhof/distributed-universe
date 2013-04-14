#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <vector>
#include <string>

namespace protocol {

	//The port master listen's on
	const char SERVER_PORT[] = "2828";

	//The port each worker will listen on for its left neighbour (base + worker id)
	const int BASE_NEIGHBOUR_PORT = 2929;

	//Backlog for master listening port
	const int SERVER_LISTEN_BACKLOG = 20;

	/**
	 * -----------------------------------------------------------------------------------------------------------------
	 * Message definitions
	 * -----------------------------------------------------------------------------------------------------------------
	 */

	/**
	 * JOIN: Sent from worker to master to initiate a connection
	 */
	const unsigned char JOIN_MESSAGE = 0x00;

	/**
	 * JOIN_ACK: Sent from master to worker following a JOIN
	 *
	 * Payload:
	 * uint32_t id             The assigned id to the worker
	 * uint32_t num_workers    The expected num_workers in total
	 */
	const unsigned char JOIN_ACK_MESSAGE = 0x01;

	/**
	 * LISTENING_FOR_NEIGHBOUR: Sent from worker to master following a JOIN_ACK to notify master that the worker is
	 * now listening on the correct port for its left neighbour
	 */
	const unsigned char LISTENING_FOR_NEIGHBOUR_MESSAGE = 0x02;

	/**
	 * RIGHT_NEIGHBOUR_DISCOVER: Sent from master to worker following a LISTENING_FOR_NEIGHBOUR to inform the worker
	 * where he can find his right neighbour (ip address)
	 *
	 * Payload:
	 * uint32_t size         The size of the following ip address
	 * char* ip_address      The ip address of the right neighbour
	 */
	const unsigned char RIGHT_NEIGHBOUR_DISCOVER_MESSAGE = 0x03;

	/**
	 * NEIGHBOURS_SET: Sent from worker to master following a RIGHT_NEIGHBOUR_DISCOVER to notify master that the worker
	 * has connections set up to both its left and right neigbours
	 */
	const unsigned char NEIGHBOURS_SET_MESSAGE = 0x04;

	/**
	 * NEIGHBOUR_REQUEST: Sent from left peer to right peer as a request to be worker neighbours
	 *
	 */
	const unsigned char NEIGHBOUR_REQUEST_MESSAGE = 0x05;

	/**
	 * NEIGHBOUR_REQUEST_ACK: Sent from right peer to left peer following a NEIGHBOUR_REQUEST message. At this point,
	 * the peers are officially neighbours
	 *
	 */
	const unsigned char NEIGHBOUR_REQUEST_ACK_MESSAGE = 0x06;

	/**
	 * SET_UNIVERSE_PARAMEERS_MESSAGE: Sent from master to worker following a NEIGHBOURS_SET_MESSAGE to inform the worker
	 * of the user provided universe parameters
	 *
	 * Payload:
	 *uint32_t world_size              The world size of the universe (side length)
	 *uint32_t robot_range             The range of a robot's fov sensor
	 *int32_t num_updates              The number of updates to run before we finish
	 *uint32_t num_blocks              The num of blocks in the grid (NxN)
	 *uint32_t visualization_enabled   Is visualization enabled (0: false, 1: true)
	 *uint32_t fov                     The robot fov in mr
	 *uint32_t invert_direction        Is robot direction inverted (0: false, 1: true)
	 */
	const unsigned char SET_UNIVERSE_PARAMETERS_MESSAGE = 0x07;

	/**
	 * UNIVERSE_PARAMETERS_SET_MESSAGE: Sent from worker to master following a SET_UNIVERSE_PARAMETERS_MESSAGE to inform
	 * the master that the worker has set the previously provided parameters
	 *
	 */
	const unsigned char UNIVERSE_PARAMETERS_SET_MESSAGE = 0x08;

	/**
	 * SET_ROBOTS_MESSAGE: Sent from master to worker following a UNIVERSE_PARAMETERS_SET_MESSAGE with a collection
	 * of robots that worker needs to add
	 *
	 * Payload:
	 * uint32_t num_robots The number of robots
	 * long serialized robots x num_robots
	 *
	 */
	const unsigned char SET_ROBOTS_MESSAGE = 0x09;

	/**
	 * ROBOTS_SET_MESSAGE: Sent from worker to master following a SET_ROBOTS_MESSAGE to notify master that the robots
	 * have been added
	 *
	 */
	const unsigned char ROBOTS_SET_MESSAGE = 0x0A;

	/**
	 * START_SIMULATION_MESSAGE: Sent from master to worker following a ROBOTS_SET_MESSAGE to tell the worker to begin
	 * the simulation
	 *
	 */
	const unsigned char START_SIMULATION_MESSAGE = 0x0B;

	/**
	 *GHOST_STRIP_MESSAGE: Sent from worker to worker with the contents of a ghost strip
	 *
	 *Payload:
	 *N of:
	 *   uint32_t x_key            The x component of a map coordinate
	 *   uint32_t y_key            The y component of a map coordinate
	 *   uint32_t num_robots       The number of robots in this block
	 *   ghost serialized robots x num_robots
	 *
	 */
	const unsigned char GHOST_STRIP_MESSAGE = 0x0C;

	/**
	 * ADD_ROBOTS_MESSAGE: Sent from worker to worker containing the robots that the worker needs to take ownership of
	 *
	 * Payload:
	 * uint32_t num_robots
	 * long serialized robots x num_robots
	 *
	 */
	const unsigned char ADD_ROBOTS_MESSAGE = 0x0D;

	/**
	 * FRAME_FINISHED_MESSAGE: Sent from worker to master after a frame has been completed
	 *
	 */
	const unsigned char FRAME_FINISHED_MESSAGE = 0x0E;

	/**
	 * FRAME_FINISHED_WITH_STATS_MESSAGE: Sent from worker to master after a frame has been completed (visualization on)
	 *
	 * Payload:
	 * num_blocks        The number of blocks
	 * num_blocks X:
	 *    uint32_t x     The x coordinate for a grid block
	 *    uint32_t y     The y coordinate for a grid block
	 *    uint32_t size  The count of robots within the block
	 *
	 */
	const unsigned char FRAME_FINISHED_WITH_STATS_MESSAGE = 0x0F;

	/**
	 * FINAL_POSITIONS_MESSAGE: Sent from worker to master after all updates have been completed to inform master of
	 * the final robot positions
	 *
	 *Payload:
	 *uint32_t num_robots
	 *normal serialized robots x num_robots
	 */
	const unsigned char FINAL_POSITIONS_MESSAGE = 0x10;

	/**
	 * -----------------------------------------------------------------------------------------------------------------
	 * End Message definitions
	 * -----------------------------------------------------------------------------------------------------------------
	 */

	//Gets the textual name for the various message types
	std::string get_message_type_name(const unsigned char message_type);

	// Retrieves the entire message from a socket stripping length header
	void recieve_message(int socket, std::vector<unsigned char>& message);

	// Send an entire message to the socket after prepending the length header
	void send_message(int socket, const std::vector<unsigned char>& message);

	// Send an entire message to the socket. Assumes you handled the length header (faster version)
	void send_message(int socket, unsigned char* message, size_t len);

}

#endif /* PROTOCOL_H_ */
