#ifndef ROBOT_MAP_H_
#define ROBOT_MAP_H_

#include <vector>
#include <inttypes.h>

#include "robot.h"

/**
 *
 * A robot map structure that maps robot x,y coordinates to a specific block within a grid
 *
 */
class RobotMap {

	private:
		uint32_t num_blocks;
		uint32_t left_x_bound;
		uint32_t right_x_bound;
		uint32_t width;
		std::vector<Robot*> ***grid;

		// Containers for robots that are no longer within our bounds. To be sent to left & right neighbours respectively
		std::vector<std::pair<MapCoordinate, Robot*>> left_neighbours_robots;
		std::vector<std::pair<MapCoordinate, Robot*>> right_neighbours_robots;

		MapCoordinate localize_coordinate(MapCoordinate coordinate);
		MapCoordinate unlocalize_coordinate(MapCoordinate coordinate);
		int32_t wrap_y_coordinate(int32_t y);

		// Compares a robot to all robots within the specified block
		void compare_robot_to_block(Robot *robot, MapCoordinate localized_coordinate);

		uint32_t send_ghost_strip_message(int fd, uint32_t ghost_x_index);

		void add_ghost_strip_robot(Robot& robot, MapCoordinate coordinate, uint32_t ghost_x_index);

		uint32_t send_moved_robots(int fd, std::vector<std::pair<MapCoordinate, Robot*>>* robots, int flag);

	public:
		RobotMap(uint32_t num_blocks, uint32_t left_x_bound, uint32_t right_x_bound);
		~RobotMap();

		void add_robot(Robot& robot);
		void add_robot(Robot& robot, MapCoordinate coordinate);

		void add_left_ghost_strip_robot(Robot& robot, MapCoordinate coordinate);
		void add_right_ghost_strip_robot(Robot& robot, MapCoordinate coordinate);

		// Updates all robot positions according to their current speed (1)
		void update_robot_positions_and_reset_sensors();

		// Updates all robot sensors by comparison to robots within this map(2)
		void update_robot_sensors();

		// Moves all robots in space according to the state of their current sensors (3)
		void set_robot_speeds_and_directions();

		void clear_ghost_strips();

		uint32_t send_left_ghost_strip_message(int fd);
		uint32_t send_right_ghost_strip_message(int fd);

		uint32_t send_left_moved_robots(int fd);
		uint32_t send_right_moved_robots(int fd);

		// Creates and sends a FRAME_FINISHED_WITH_STATS_MESSAGE using the contents of the map
		void send_frame_stats_message(int fd);

		// Creates and sends a FINAL_POSITIONS_MESSAGE using the contents of the map
		void send_final_positions_message(int fd);

		void dump_map();

};

#endif /* ROBOT_MAP_H_ */
