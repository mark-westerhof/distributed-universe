#include "robot_map.h"

#include "netutils.h"
#include "protocol.h"

RobotMap::RobotMap(uint32_t num_blocks, uint32_t left_x_bound, uint32_t right_x_bound) {
	this->num_blocks = num_blocks;
	this->left_x_bound = left_x_bound;
	this->right_x_bound = right_x_bound;
	width = right_x_bound - left_x_bound + 1;

	grid = new std::vector<Robot*>**[num_blocks];
	for (unsigned int y = 0; y < num_blocks; y++) {
		grid[y] = new std::vector<Robot*>*[width + 2];
		for (unsigned int x = 0; x < width + 2; x++) {
			grid[y][x] = new std::vector<Robot*>();
		}
	}
}

RobotMap::~RobotMap() {
}

MapCoordinate RobotMap::localize_coordinate(MapCoordinate coordinate) {
	return MapCoordinate(coordinate.first - left_x_bound + 1, coordinate.second);
}

MapCoordinate RobotMap::unlocalize_coordinate(MapCoordinate coordinate) {
	return MapCoordinate(coordinate.first + left_x_bound - 1, coordinate.second);
}

int32_t RobotMap::wrap_y_coordinate(int32_t y) {
	int64_t num_blocks_unsigned = num_blocks;
	if (y < 0) {
		y = num_blocks_unsigned - 1;
	} else if (y > num_blocks_unsigned - 1) {
		y = 0;
	}
	return y;
}

void RobotMap::add_robot(Robot& robot) {
	add_robot(robot, robot.calc_map_coordinate(num_blocks));
}

void RobotMap::add_robot(Robot& robot, MapCoordinate coordinate) {
	MapCoordinate localized = localize_coordinate(coordinate);
	grid[localized.second][localized.first]->push_back(&robot);
}

void RobotMap::update_robot_positions_and_reset_sensors() {
	std::vector<std::pair<MapCoordinate, Robot*>> moved_robots;

	for (unsigned int y = 0; y < num_blocks; y++) {
		for (unsigned int x = 1; x <= width; x++) {
			auto it = grid[y][x]->begin();
			while (it != grid[y][x]->end()) {
				Robot *robot = *it;

				//What block are we currently in?
				MapCoordinate coordinate = robot->calc_map_coordinate(num_blocks);

				MapCoordinate new_coordinate = robot->update_position_and_reset_sensors(num_blocks);
				if (coordinate.first != new_coordinate.first || coordinate.second != new_coordinate.second) {
					it = grid[y][x]->erase(it);
					moved_robots.push_back(std::pair<MapCoordinate, Robot*>(new_coordinate, robot));
				} else {
					++it;
				}
			}
		}
	}

	//Decide where the robots should go
	left_neighbours_robots.clear();
	right_neighbours_robots.clear();
	for (unsigned int i = 0; i < moved_robots.size(); i++) {
		MapCoordinate coordinate = moved_robots.at(i).first;
		Robot *robot = moved_robots.at(i).second;

		if (coordinate.first < left_x_bound) {

			// Right world wrap around
			if ((right_x_bound == num_blocks - 1) && coordinate.first == 0) {
				right_neighbours_robots.push_back(std::pair<MapCoordinate, Robot*>(coordinate, robot));
			}
			// Normal case
			else {
				left_neighbours_robots.push_back(std::pair<MapCoordinate, Robot*>(coordinate, robot));
			}
		}

		else if (coordinate.first > right_x_bound) {

			// Left world wrap around
			if (left_x_bound == 0 && (coordinate.first == num_blocks - 1)) {
				left_neighbours_robots.push_back(std::pair<MapCoordinate, Robot*>(coordinate, robot));
			}
			// Normal case
			else {
				right_neighbours_robots.push_back(std::pair<MapCoordinate, Robot*>(coordinate, robot));
			}
		}

		// It hasn't left our local map
		else {
			add_robot(*robot, coordinate);
		}
	}
}

void RobotMap::update_robot_sensors() {

	for (unsigned int y = 0; y < num_blocks; y++) {
		for (unsigned int x = 1; x <= width; x++) {

			//What other neighbouring blocks do we need to compare to?
			MapCoordinate localized_neighbours[9];

			uint32_t top_y = wrap_y_coordinate(y - 1);
			localized_neighbours[0] = MapCoordinate(x - 1, top_y);
			localized_neighbours[1] = MapCoordinate(x, top_y);
			localized_neighbours[2] = MapCoordinate(x + 1, top_y);

			localized_neighbours[3] = MapCoordinate(x - 1, y);
			localized_neighbours[4] = MapCoordinate(x, y);
			localized_neighbours[5] = MapCoordinate(x + 1, y);

			uint32_t bottom_y = wrap_y_coordinate(y + 1);
			localized_neighbours[6] = MapCoordinate(x - 1, bottom_y);
			localized_neighbours[7] = MapCoordinate(x, bottom_y);
			localized_neighbours[8] = MapCoordinate(x + 1, bottom_y);

			for (auto it = grid[y][x]->begin(); it != grid[y][x]->end(); ++it) {
				for (unsigned int i = 0; i < 9; i++) {
					compare_robot_to_block(*it, localized_neighbours[i]);
				}
			}
		}
	}
}

void RobotMap::compare_robot_to_block(Robot *robot, MapCoordinate localized_coordinate) {
	std::vector<Robot*> *vector = grid[localized_coordinate.second][localized_coordinate.first];
	for (auto it = vector->begin(); it != vector->end(); ++it) {
		robot->update_sensors(**it);
	}
}

void RobotMap::set_robot_speeds_and_directions() {
	for (unsigned int y = 0; y < num_blocks; y++) {
		for (unsigned int x = 1; x <= width; x++) {
			for (auto it = grid[y][x]->begin(); it != grid[y][x]->end(); ++it) {
				(*it)->set_speed_and_direction();
			}
		}
	}
}

void RobotMap::clear_ghost_strips() {
	for (unsigned int y = 0; y < num_blocks; y++) {
		uint32_t x_ghost_indexes[] = { 0, width + 1 };
		for (unsigned int x_index = 0; x_index < 2; x_index++) {
			std::vector<Robot*> *vector = grid[y][x_ghost_indexes[x_index]];
			for (auto it = vector->begin(); it != vector->end(); ++it) {
				delete *it;
			}
			vector->clear();
		}
	}
}

uint32_t RobotMap::send_ghost_strip_message(int fd, uint32_t ghost_x_index) {
	// Get the total count
	uint32_t total_robot_count = 0;
	for (unsigned int y = 0; y < num_blocks; y++) {
		total_robot_count += grid[y][ghost_x_index]->size();
	}

	// Create the message
	uint32_t message_size = (num_blocks * 12) + (total_robot_count * Robot::GHOST_SERIALIZED_LENGTH) + 5;
	unsigned char message[message_size];
	netutils::insert_uint32_into_message(message_size - 4, message);
	message[4] = protocol::GHOST_STRIP_MESSAGE;
	uint64_t message_index = 5;

	uint32_t count = 0;
	for (unsigned int y = 0; y < num_blocks; y++) {
		MapCoordinate coordinate = unlocalize_coordinate(MapCoordinate(ghost_x_index, y));
		uint32_t block_count = grid[y][ghost_x_index]->size();

		//Add the coordinate
		netutils::insert_uint32_into_message(coordinate.first, &message[message_index]);
		netutils::insert_uint32_into_message(coordinate.second, &message[message_index + 4]);
		netutils::insert_uint32_into_message(block_count, &message[message_index + 8]);
		message_index += 12;

		for (unsigned int i = 0; i < block_count; i++) {
			Robot *robot = grid[y][ghost_x_index]->at(i);
			robot->serialize_ghost(&message[message_index]);
			message_index += Robot::GHOST_SERIALIZED_LENGTH;
			count++;
		}
	}
	protocol::send_message(fd, message, message_size);
	return count;
}

uint32_t RobotMap::send_left_ghost_strip_message(int fd) {
	return send_ghost_strip_message(fd, 1);
}
uint32_t RobotMap::send_right_ghost_strip_message(int fd) {
	return send_ghost_strip_message(fd, width);
}

void RobotMap::add_ghost_strip_robot(Robot& robot, MapCoordinate coordinate, uint32_t ghost_x_index) {
	grid[coordinate.second][ghost_x_index]->push_back(&robot);
}

void RobotMap::add_left_ghost_strip_robot(Robot& robot, MapCoordinate coordinate) {
	add_ghost_strip_robot(robot, coordinate, 0);
}
void RobotMap::add_right_ghost_strip_robot(Robot& robot, MapCoordinate coordinate) {
	add_ghost_strip_robot(robot, coordinate, width + 1);
}

uint32_t RobotMap::send_moved_robots(int fd, std::vector<std::pair<MapCoordinate, Robot*>>* robots, int flag) {
	uint32_t message_size = 9 + (robots->size() * Robot::LONG_SERIALIZED_LENGTH);
	unsigned char send_message[message_size];
	netutils::insert_uint32_into_message(message_size - 4, send_message);
	send_message[4] = protocol::ADD_ROBOTS_MESSAGE;
	netutils::insert_uint32_into_message(robots->size(), &send_message[5]);

	for (unsigned int i = 0; i < robots->size(); i++) {
		MapCoordinate coordinate = robots->at(i).first;
		Robot *robot = robots->at(i).second;
		robot->serialize_long(&send_message[9 + (i * Robot::LONG_SERIALIZED_LENGTH)]);

		// Tricky: Insert these into our ghost strip. We held off sending these before ghost strip exchanges to avoid
		// the overhead of getting them right back in the respective ghost strip
		if (flag == 0) {
			add_left_ghost_strip_robot(*robot, coordinate);
		} else {
			add_right_ghost_strip_robot(*robot, coordinate);
		}
	}
	protocol::send_message(fd, send_message, message_size);
	return robots->size();
}

uint32_t RobotMap::send_left_moved_robots(int fd) {
	return send_moved_robots(fd, &left_neighbours_robots, 0);
}
uint32_t RobotMap::send_right_moved_robots(int fd) {
	return send_moved_robots(fd, &right_neighbours_robots, 1);
}

void RobotMap::send_final_positions_message(int fd) {
	uint32_t num_robots = 0;
	for (unsigned int y = 0; y < num_blocks; y++) {
		for (unsigned int x = 1; x <= width; x++) {
			num_robots += grid[y][x]->size();
		}
	}

	uint32_t message_size = 9 + (num_robots * Robot::NORMAL_SERIALIZED_LENGTH);
	unsigned char message[message_size];
	netutils::insert_uint32_into_message(message_size - 4, message);
	message[4] = protocol::FINAL_POSITIONS_MESSAGE;
	netutils::insert_uint32_into_message(num_robots, &message[5]);

	unsigned int index = 0;
	for (unsigned int y = 0; y < num_blocks; y++) {
		for (unsigned int x = 1; x <= width; x++) {
			for (unsigned int i = 0; i < grid[y][x]->size(); i++) {
				grid[y][x]->at(i)->serialize_normal(&message[9 + (index * Robot::NORMAL_SERIALIZED_LENGTH)]);
				index++;
			}
		}
	}
	protocol::send_message(fd, message, message_size);
}

void RobotMap::send_frame_stats_message(int fd) {
	uint32_t total_local_blocks = num_blocks * width;
	uint32_t message_size = 9 + (total_local_blocks * 12);
	unsigned char message[message_size];
	netutils::insert_uint32_into_message(message_size - 4, message);
	message[4] = protocol::FRAME_FINISHED_WITH_STATS_MESSAGE;
	netutils::insert_uint32_into_message(total_local_blocks, &message[5]);

	uint32_t block_count = 0;
	for (uint32_t y = 0; y < num_blocks; y++) {
		for (uint32_t x = 1; x <= width; x++) {
			MapCoordinate coordinate = unlocalize_coordinate(MapCoordinate(x, y));
			uint32_t offset = block_count * 12;
			netutils::insert_uint32_into_message(coordinate.first, &message[9 + offset]);
			netutils::insert_uint32_into_message(coordinate.second, &message[13 + offset]);
			netutils::insert_uint32_into_message(grid[y][x]->size(), &message[17 + offset]);
			block_count++;
		}
	}
	protocol::send_message(fd, message, message_size);
}

void RobotMap::dump_map() {
	printf("Left bound: %u\n", left_x_bound);
	printf("Right bound: %u\n", right_x_bound);
	printf("Width: %u\n", width);
	for (unsigned int y = 0; y < num_blocks; y++) {
		for (unsigned int x = 0; x < width + 2; x++) {
			printf("Local Block %u,%u:\n", x, y);
			for (unsigned int i = 0; i < grid[y][x]->size(); i++) {
				Robot *robot = grid[y][x]->at(i);
				MapCoordinate coordinate = robot->calc_map_coordinate(num_blocks);
				printf("   (%u,%u) - %s\n", coordinate.first, coordinate.second, robot->to_string_long().c_str());
			}
		}
	}
}

