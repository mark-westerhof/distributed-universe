#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include "inttypes.h"
#include "pthread.h"

/**
 *
 * Distributed visualization. Namespaced due to lack of OOD in glfw
 *
 */
namespace visualization {

	extern uint32_t _num_blocks;
	extern uint32_t _population;
	extern uint32_t **_block_values;
	extern uint32_t _num_blocks_sets;
	extern uint32_t _max_block_value;
	extern uint32_t _min_block_value;

	extern pthread_mutex_t _lock;
	extern pthread_cond_t _can_draw;
	extern bool _wait_to_draw;

	void start(uint32_t num_blocks, uint32_t size);

	void* _entry(void* arg);
	void _loop();
	void _draw();

	void set_block_stat(uint32_t x, uint32_t y, uint32_t size);
}

#endif /* VISUALIZATION_H_ */
