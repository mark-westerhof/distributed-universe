#include "arguments.h"

#include <unistd.h>
#include <cstdio>
#include <cstdlib>

#include "robot.h"

Arguments::Arguments(int argc, char **argv) {
	// Set default options
	num_updates = Arguments::DEFAULT_NUM_UPDATES;
	world_size = Arguments::DEFAULT_WORLD_SIZE;
	robot_range = Arguments::DEFAULT_ROBOT_RANGE;
	num_blocks = Arguments::DEFAULT_NUM_BLOCKS;
	fov = Robot::millidegrees_to_milliradians(Arguments::DEFAULT_FOV * 1000);
	direction_inverted = Arguments::DEFAULT_INVERT_DIRECTION;
	worker_debug_enabled = Arguments::DEFAULT_WORKER_DEBUG_ENABLED;
	visualization_enabled = Arguments::DEFAULT_VISUALIZATION_ENABLED;

	bool num_workers_provided = false;
	bool population_size_provided = false;

	int c;
	while ((c = getopt(argc, argv, "hn:p:u:s:r:b:f:idv")) != -1) {
		switch (c) {
			case 'h':
				print_usage(argv);
				print_help();
				exit (EXIT_SUCCESS);
				break;

			case 'n':
				num_workers = atoi(optarg);
				if (num_workers < 2) {
					fprintf(stderr, "Number of workers must be >= 2\n");
					exit (EXIT_FAILURE);
				}
				num_workers_provided = true;
				break;

			case 'p':
				population_size = atoi(optarg);
				if (population_size < 1) {
					fprintf(stderr, "Population size must be >= 1\n");
					exit (EXIT_FAILURE);
				}
				population_size_provided = true;
				break;

			case 'u':
				num_updates = atoi(optarg);
				if (num_updates < 0) {
					fprintf(stderr, "Number of updates must be >= 0\n");
					exit (EXIT_FAILURE);
				}
				break;

			case 's':
				world_size = atoi(optarg);
				if (world_size < 100) {
					fprintf(stderr, "World size must be >= 100\n");
					exit (EXIT_FAILURE);
				}
				break;

			case 'r':
				robot_range = atoi(optarg);
				if (robot_range < 1) {
					fprintf(stderr, "Robot range must be >= 1\n");
					exit (EXIT_FAILURE);
				}
				break;

			case 'b':
				num_blocks = atoi(optarg);
				if (num_blocks < 1) {
					fprintf(stderr, "Number of blocks must be >= 1\n");
					exit (EXIT_FAILURE);
				}
				break;

			case 'f':
				fov = Robot::millidegrees_to_milliradians(atoi(optarg) * 1000);
				if (fov <= 0) {
					fprintf(stderr, "Field of view must be > 0\n");
					exit (EXIT_FAILURE);
				}
				break;

			case 'i':
				direction_inverted = true;
				break;

			case 'd':
				worker_debug_enabled = true;
				break;

			case 'v':
				visualization_enabled = true;
				break;

			default:
				print_usage(argv);
				exit (EXIT_FAILURE);
				break;
		}
	}

	//Check that required arguments were supplied
	if (!num_workers_provided) {
		fprintf(stderr, "Number of workers not supplied\n");
		exit (EXIT_FAILURE);
	}
	if (!population_size_provided) {
		fprintf(stderr, "Population size not supplied\n");
		exit (EXIT_FAILURE);
	}

	validate_block_size();
	validate_num_workers();
}

Arguments::~Arguments() {
}

void Arguments::validate_block_size() {

	// Calculate the max number of blocks possible that the space can be evenly divided into
	int32_t max_num_blocks = world_size / robot_range;
	while (world_size % max_num_blocks != 0) {
		max_num_blocks--;
	}

	//User didn't provide, use max possible
	if (num_blocks < 0) {
		num_blocks = max_num_blocks;
	}
	// Validate user provided value
	else {
		//Is the space divisble by the number of blocks?
		if (world_size % num_blocks) {
			fprintf(stderr, "The 2D space (world size) of '%d' cannot be evenly divided into '%dx%d' blocks\n",
					world_size, num_blocks, num_blocks);
			exit (EXIT_FAILURE);
		}

		if (num_blocks > max_num_blocks) {
			fprintf(stderr, "The number of blocks can be no greater than %d for the desired configuration\n",
					max_num_blocks);
			exit (EXIT_FAILURE);
		}
	}
}

void Arguments::validate_num_workers() {

	bool invalid = false;
	int32_t block_size = world_size / num_blocks;
	int32_t slice_size = world_size / num_workers;

	// Check if we can divide the 2D space into N worker slices evenly
	if (world_size % num_workers != 0) {
		fprintf(stderr, "The 2D space (world size) of '%d' cannot be evenly divided into '%d' worker slices\n",
				world_size, num_workers);
		invalid = true;
	}
	// Check if a slice is evenly divisible by a block
	else if (slice_size % block_size != 0) {
		fprintf(stderr,
				"A worker slice of size '%d' (for '%d' workers) is not evenly divisible by a block of size '%d'\n"
						"('%dx%d' blocks in the 2D space of '%d')\n", slice_size, num_workers, block_size, num_blocks,
				num_blocks, world_size);
		invalid = true;
	}

	if (invalid) {
		fprintf(stderr, "Valid number of workers for the desired configuration are: ");
		for (int i = 2; i <= num_blocks; i++) {
			if (world_size % i == 0) {
				int32_t slice_size = world_size / i;
				if (slice_size % block_size == 0) {
					fprintf(stderr, "%d", i);
					if (i != num_blocks) {
						fprintf(stderr, ", ");
					}
				}
			}
		}
		fprintf(stderr, "\n");
		exit (EXIT_FAILURE);
	}
}

void Arguments::print_usage(char **argv) {
	static const char usage[] = "Usage: %s [OPTION] -n num_workers -p pop_size\n";
	printf(usage, argv[0]);
}

void Arguments::print_help() {
	static const char mandatory_args[] = "Mandatory arguments:\n"
			"  -n num_workers   The number of worker nodes to use\n"
			"  -p pop_size      The number of robots in the universe\n";

	static const char optional_args[] =
			"Optional arguments:\n"
					"  -u num_updates   The number of updates to run before quitting [Default: No limit]\n"
					"  -s world_size    The side length of the (square) world [Default: 1000]\n"
					"  -r robot_range   A robot's sensor field of view range [Default: 100]\n"
					"  -b num_blocks    The number of blocks to subdivide the 2D space into (NxN) [Default: Maximum possible]\n"
					"  -f fov           The field of view of a robot's sensors in degrees [Default: 270]\n"
					"  -i               Invert robot direction behavior. Move toward others instead of away [Default: no]\n"
					"  -d               Enable worker debugging to identify a slow worker (in combination with '-u') [Default: no]\n"
					"  -v               Enable visualization [Default: no]\n";

	puts(mandatory_args);
	puts(optional_args);
}

void Arguments::print_arguments() {
	uint32_t slice_size = get_worker_slice_size();
	uint32_t block_size = get_block_size();

	printf("**************************************************\n");
	printf("Universe Parameters:\n");
	printf("   Population size:    %d\n", population_size);
	printf("   World size:         %d\n", world_size);
	printf("   Robot range:        %d\n", robot_range);
	printf("   Robot FOV:          %d mrad (%d deg)\n", fov, Robot::milliradians_to_millidegrees(fov) / 1000);
	printf("   Inverted:           %s\n", direction_inverted ? "Yes" : "No");
	if (num_updates < 0) {
		printf("   Number of updates:  No limit\n");
	} else {
		printf("   Number of updates:  %d\n", num_updates);
	}
	printf("Distribution Configuration:\n");
	printf("   Grid size:          %dx%d\n", num_blocks, num_blocks);
	printf("   Number of workers:  %d\n", num_workers);
	printf("   Worker slice size:  %d (%dx%d blocks)\n", slice_size, slice_size / block_size, num_blocks);
	printf("   Worker debugging:   %s\n", worker_debug_enabled ? "Yes" : "No");
	printf("   Visualization:      %s\n", visualization_enabled ? "Yes" : "No");
	printf("**************************************************\n");
}

int32_t Arguments::get_num_updates() {
	return num_updates;
}

uint32_t Arguments::get_population_size() {
	return population_size;
}

uint32_t Arguments::get_world_size() {
	return world_size;
}

uint32_t Arguments::get_robot_range() {
	return robot_range;
}

uint32_t Arguments::get_num_blocks() {
	return num_blocks;
}

uint32_t Arguments::get_block_size() {
	return world_size / num_blocks;
}

uint32_t Arguments::get_num_workers() {
	return num_workers;
}

uint32_t Arguments::get_worker_slice_size() {
	return world_size / num_workers;
}

uint32_t Arguments::get_fov() {
	return fov;
}

bool Arguments::is_direction_inverted() {
	return direction_inverted;
}

bool Arguments::is_worker_debug_enabled() {
	return worker_debug_enabled;
}

bool Arguments::is_visualization_enabled() {
	return visualization_enabled;
}

