#ifndef ROBOT_H_
#define ROBOT_H_

#include <inttypes.h>
#include <string>

typedef std::pair<uint32_t, uint32_t> MapCoordinate;

/**
 *
 * Robot that moves in 2 dimensional space
 *
 */
class Robot {

	private:

		static const int32_t THOUSAND_TIMES_PI = 3142;
		static const int32_t NUM_PIXELS = 8;

		uint32_t id;
		int32_t current_closest_range;
		int32_t closest_pixel;

		int32_t x_position;
		int32_t y_position;
		int32_t a_position;

		int32_t linear_speed;
		int32_t angular_speed;

		static uint32_t id_count;
		static int32_t world_size;
		static int32_t half_world_size;
		static int32_t fov;
		static int32_t milliradians_per_pixel;

		static int32_t normalize_angle(int32_t angle);
		static int32_t normalize_distance(int32_t distance);

	public:

		static int32_t range;
		static bool invert_direction;

		static const int NORMAL_SERIALIZED_VERSION = 0;
		static const int LONG_SERIALIZED_VERSION = 1;
		static const int GHOST_SERIALIZED_VERSION = 2;

		// How long are the serialized version of robots in bytes?
		static const int NORMAL_SERIALIZED_LENGTH = 16;
		static const int LONG_SERIALIZED_LENGTH = 24;
		static const int GHOST_SERIALIZED_LENGTH = 8;

		// Default constructor (for first time only)
		Robot();

		// Constructor for a serialized robot
		Robot(unsigned char* location, int serialized_version);

		~Robot();

		// Get only the robot id from a serialized version
		static uint32_t get_id_from_serialized(unsigned char* location);

		// Updates existing robot from a serialized version
		void update_from_serialized(unsigned char* location, int serialized_version);

		int32_t get_x_position();
		int32_t get_y_position();

		// Calculates the key in which this robot should fall under for its current position
		MapCoordinate calc_map_coordinate(uint32_t num_blocks);

		static void set_world_size(int32_t world_size);
		static uint32_t get_world_size();
		static void set_fov(int32_t fov);

		// Wrap around the torus an X or Y coordinate
		static int32_t wrap_around_coordinate(int32_t coordinate);

		static int32_t millidegrees_to_milliradians(int32_t millidegrees);
		static int32_t milliradians_to_millidegrees(int32_t milliradians);

		// Updates robot position according to the current speed, and resets its sensors (1)
		// Returns the new MapCoordinate the robot now falls under
		MapCoordinate update_position_and_reset_sensors(uint32_t num_blocks);

		// Updates sensors by comparison to a given robot (2)
		void update_sensors(Robot &robot);

		// Moves in space according to the state of its current sensors (3)
		void set_speed_and_direction();

		// Serializes the robot (in normal, long, and ghost form respectively) to the desired location in a message
		void serialize_normal(unsigned char* location);
		void serialize_long(unsigned char* location);
		void serialize_ghost(unsigned char* location);

		std::string to_string_short();
		std::string to_string_long();
};

#endif /* ROBOT_H_ */
