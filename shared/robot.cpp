#include "robot.h"

#include <utility>
#include <cstdlib>
#include <cstdio>
#include <math.h>

#include "netutils.h"

//Declare static members
uint32_t Robot::id_count = 1;

//Default values
int32_t Robot::world_size = 1000;
int32_t Robot::half_world_size = 500;
int32_t Robot::fov = Robot::millidegrees_to_milliradians(270000);
int32_t Robot::milliradians_per_pixel = Robot::fov / Robot::NUM_PIXELS;
bool Robot::invert_direction = false;
int32_t Robot::range = 100;

Robot::Robot() {
	id = Robot::id_count++;
	current_closest_range = Robot::range;
	closest_pixel = -1;
	linear_speed = 0;
	angular_speed = 0;

	//Set our random position
	x_position = drand48() * world_size;
	y_position = drand48() * world_size;
	a_position = normalize_angle(drand48() * (THOUSAND_TIMES_PI * 2.0));
}

Robot::Robot(unsigned char* location, int serialized_version) {
	switch (serialized_version) {
		case NORMAL_SERIALIZED_VERSION:
			id = netutils::get_uint32_from_message(location);
			x_position = netutils::get_uint32_from_message(location + 4);
			y_position = netutils::get_uint32_from_message(location + 8);
			a_position = netutils::get_uint32_from_message(location + 12);
			linear_speed = 0;
			angular_speed = 0;
			break;

		case LONG_SERIALIZED_VERSION:
			id = netutils::get_uint32_from_message(location);
			x_position = netutils::get_uint32_from_message(location + 4);
			y_position = netutils::get_uint32_from_message(location + 8);
			a_position = netutils::get_uint32_from_message(location + 12);
			linear_speed = netutils::get_uint32_from_message(location + 16);
			angular_speed = netutils::get_uint32_from_message(location + 20);
			break;

		case GHOST_SERIALIZED_VERSION:
			id = 0;
			x_position = netutils::get_uint32_from_message(location);
			y_position = netutils::get_uint32_from_message(location + 4);
			a_position = 0;
			linear_speed = 0;
			angular_speed = 0;
			break;

		default:
			fprintf(stderr, "Invalid serialization version\n");
			exit(EXIT_FAILURE);
	}

	current_closest_range = Robot::range;
	closest_pixel = -1;
}

Robot::~Robot() {
}

uint32_t Robot::get_id_from_serialized(unsigned char* location) {
	return netutils::get_uint32_from_message(location);
}

// Updates existing robot from a serialized version
void Robot::update_from_serialized(unsigned char* location, int serialized_version) {
	switch (serialized_version) {
		case NORMAL_SERIALIZED_VERSION:
			x_position = netutils::get_uint32_from_message(location + 4);
			y_position = netutils::get_uint32_from_message(location + 8);
			a_position = netutils::get_uint32_from_message(location + 12);
			break;

		default:
			fprintf(stderr, "Invalid serialization version\n");
			exit(EXIT_FAILURE);
			break;
	}
}

int32_t Robot::get_x_position() {
	return x_position;
}
int32_t Robot::get_y_position() {
	return y_position;
}

MapCoordinate Robot::calc_map_coordinate(uint32_t num_blocks) {
	uint32_t x_key;
	if (x_position == Robot::world_size) {
		x_key = num_blocks - 1;
	} else {
		x_key = ((float) x_position / Robot::world_size) * num_blocks;
	}

	uint32_t y_key;
	if (y_position == Robot::world_size) {
		y_key = num_blocks - 1;
	} else {
		y_key = ((float) y_position / Robot::world_size) * num_blocks;
	}
	return MapCoordinate(x_key, y_key);
}

void Robot::set_world_size(int32_t world_size) {
	Robot::world_size = world_size;
	Robot::half_world_size = world_size / 2;
}

uint32_t Robot::get_world_size() {
	return Robot::world_size;
}

void Robot::set_fov(int32_t fov) {
	Robot::fov = fov;
	Robot::milliradians_per_pixel = fov / Robot::NUM_PIXELS;
}

int32_t Robot::normalize_angle(int32_t angle) {
	while (angle < -THOUSAND_TIMES_PI) {
		angle += 2.0 * THOUSAND_TIMES_PI;
	}
	while (angle > THOUSAND_TIMES_PI) {
		angle -= 2.0 * THOUSAND_TIMES_PI;
	}
	return angle;
}

int32_t Robot::normalize_distance(int32_t distance) {
	while (distance < 0) {
		distance += Robot::world_size;
	}
	while (distance > Robot::world_size) {
		distance -= Robot::world_size;
	}
	return distance;
}

MapCoordinate Robot::update_position_and_reset_sensors(uint32_t num_blocks) {
	// Update our position
	int32_t dx = linear_speed * cos(a_position / 1000.0);
	int32_t dy = linear_speed * sin(a_position / 1000.0);
	int32_t da = angular_speed;
	x_position = normalize_distance(x_position + dx);
	y_position = normalize_distance(y_position + dy);
	a_position = normalize_angle(a_position + da);

	// Reset our sensors
	current_closest_range = Robot::range;
	closest_pixel = -1;

	return calc_map_coordinate(num_blocks);
}

void Robot::update_sensors(Robot &robot) {

	// Ignore if it's the same robot
	if (id == robot.id) {
		return;
	}

	int32_t dx = robot.x_position - x_position;
	dx = Robot::wrap_around_coordinate(dx);
	if (abs(dx) > current_closest_range) {
		return;
	}

	int32_t dy = robot.y_position - y_position;
	dy = Robot::wrap_around_coordinate(dy);
	if (abs(dy) > current_closest_range) {
		return;
	}

	int32_t range = hypot(dx, dy);
	if (range > current_closest_range) {
		return;
	}

	// Is it in our field of view?
	int32_t absolute_heading = atan2(dy, dx) * 1000;
	int32_t relative_heading = normalize_angle((absolute_heading - a_position));
	if (abs(relative_heading) > Robot::fov / 2) {
		return;
	}

	// Which pixel does it fall into?
	relative_heading += Robot::fov / 2;
	int32_t possible_closest_pixel = relative_heading / Robot::milliradians_per_pixel;
	possible_closest_pixel %= Robot::NUM_PIXELS;

	// If the range is the same compared to our current closest, only add if the pixel that the robot in question falls
	// into is lower than the current pixel. This is silly but necessary to stay consistent with the original
	// implementation's integer arithmetic
	if (current_closest_range == range && possible_closest_pixel > closest_pixel) {
		return;
	}

	//This is now our closest
	current_closest_range = range;
	closest_pixel = possible_closest_pixel;
}

void Robot::set_speed_and_direction() {
	linear_speed = 5;
	angular_speed = 0;

	//Nothing nearby, cruise...
	if (closest_pixel < 0) {
		return;
	}
	if (closest_pixel < (int) Robot::NUM_PIXELS / 2) {
		angular_speed = 40; // Rotate right
	} else {
		angular_speed = -40; // Rotate left
	}

	//Invert if applicable
	if (Robot::invert_direction) {
		angular_speed *= -1;
	}
}

int32_t Robot::wrap_around_coordinate(int32_t coordinate) {
	if (coordinate > Robot::half_world_size) {
		coordinate -= Robot::world_size;
	} else if (coordinate < -Robot::half_world_size) {
		coordinate += Robot::world_size;
	}
	return coordinate;
}

int32_t Robot::millidegrees_to_milliradians(int32_t millidegrees) {
	return (millidegrees * Robot::THOUSAND_TIMES_PI / 180000);
}

int32_t Robot::milliradians_to_millidegrees(int32_t milliradians) {
	return (milliradians * 180000 / Robot::THOUSAND_TIMES_PI);
}

void Robot::serialize_normal(unsigned char* location) {
	netutils::insert_uint32_into_message(id, location);
	netutils::insert_uint32_into_message(x_position, location + 4);
	netutils::insert_uint32_into_message(y_position, location + 8);
	netutils::insert_uint32_into_message(a_position, location + 12);
}
void Robot::serialize_long(unsigned char* location) {
	netutils::insert_uint32_into_message(id, location);
	netutils::insert_uint32_into_message(x_position, location + 4);
	netutils::insert_uint32_into_message(y_position, location + 8);
	netutils::insert_uint32_into_message(a_position, location + 12);
	netutils::insert_uint32_into_message(linear_speed, location + 16);
	netutils::insert_uint32_into_message(angular_speed, location + 20);
}

void Robot::serialize_ghost(unsigned char* location) {
	netutils::insert_uint32_into_message(x_position, location);
	netutils::insert_uint32_into_message(y_position, location + 4);
}

std::string Robot::to_string_short() {
	std::string string = std::to_string(x_position);
	string += ",";
	string += std::to_string(y_position);
	string += ",";
	string += std::to_string(a_position);
	return string;
}

std::string Robot::to_string_long() {
	std::string string = std::to_string(id);
	string += ",";
	string += std::to_string(x_position);
	string += ",";
	string += std::to_string(y_position);
	string += ",";
	string += std::to_string(a_position);
	string += ",";
	string += std::to_string(linear_speed);
	string += ",";
	string += std::to_string(angular_speed);
	return string;
}
