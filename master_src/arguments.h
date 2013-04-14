#ifndef ARGUMENTS_H_
#define ARGUMENTS_H_

#include <inttypes.h>

/**
 *
 * Parses and encapsulates command line arguments for universe & distribution specifications
 *
 */
class Arguments {

	private:

		// -1 = No limit
		static const int32_t DEFAULT_NUM_UPDATES = -1;
		static const int32_t DEFAULT_WORLD_SIZE = 1000;
		static const int32_t DEFAULT_ROBOT_RANGE = 100;
		// -1 = Maximum possible
		static const int32_t DEFAULT_NUM_BLOCKS = -1;
		static const int32_t DEFAULT_FOV = 270;
		static const bool DEFAULT_INVERT_DIRECTION = false;
		static const bool DEFAULT_WORKER_DEBUG_ENABLED = false;
		static const bool DEFAULT_VISUALIZATION_ENABLED = false;

		int32_t num_updates;
		int32_t population_size;
		int32_t world_size;
		int32_t robot_range;
		int32_t num_blocks;
		int32_t num_workers;
		int32_t fov;
		bool direction_inverted;
		bool worker_debug_enabled;
		bool visualization_enabled;

		static void print_usage(char **argv);
		static void print_help();

		//Ensure the block size is no smaller than a robot's range
		void validate_block_size();

		//Checks if we can split the grid into N slices (N workers) where slice boundaries fall on block boundaries
		void validate_num_workers();

	public:
		Arguments(int argc, char **argv);
		~Arguments();

		void print_arguments();

		//Returns -1 if there is no update limit
		int32_t get_num_updates();

		uint32_t get_population_size();

		uint32_t get_world_size();

		uint32_t get_robot_range();

		uint32_t get_num_blocks();

		uint32_t get_block_size();

		uint32_t get_num_workers();

		uint32_t get_worker_slice_size();

		uint32_t get_fov();

		bool is_direction_inverted();

		bool is_worker_debug_enabled();

		bool is_visualization_enabled();
};

#endif /* ARGUMENTS_H_ */
