#include "visualization.h"

#include <GL/glfw.h>

#include <cstdio>
#include <cstdlib>

namespace visualization {

	uint32_t _num_blocks = 0;
	uint32_t _population = 0;
	uint32_t **_block_values;
	uint32_t _num_blocks_sets = 0;
	uint32_t _max_block_value = 0;
	uint32_t _min_block_value = 0;

	pthread_mutex_t _lock;
	pthread_cond_t _can_draw;
	bool _wait_to_draw = true;

	void start(uint32_t num_blocks, uint32_t population) {
		_num_blocks = num_blocks;
		_population = population;
		_min_block_value = population;

		_block_values = new uint32_t*[num_blocks];
		for (unsigned int i = 0; i < num_blocks; i++) {
			_block_values[i] = new uint32_t[num_blocks];
			for (unsigned int j = 0; j < num_blocks; j++) {
				_block_values[i][j] = 0;
			}
		}

		if (pthread_mutex_init(&_lock, NULL) != 0) {
			fprintf(stderr, "[Err] Failed to initialize mutexe\n");
			exit(EXIT_FAILURE);
		}

		if (pthread_cond_init(&_can_draw, NULL) != 0) {
			fprintf(stderr, "[Err] Failed to initialize condition\n");
			exit(EXIT_FAILURE);
		}

		pthread_t thread;
		if (pthread_create(&thread, NULL, &_entry, NULL) != 0) {
			fprintf(stderr, "[Err] Failed to create visualization thread\n");
			exit(EXIT_FAILURE);
		}
	}

	void* _entry(void* arg) {
		if (glfwInit() != GL_TRUE) {
			fprintf(stderr, "Failed to initialize glfw\n");
			exit(EXIT_FAILURE);
		}
		if (glfwOpenWindow(800, 800, 0, 0, 0, 0, 0, 0, GLFW_WINDOW) != GL_TRUE) {
			fprintf(stderr, "Failed to open glfw window\n");
			exit(EXIT_FAILURE);
		}
		glfwSetWindowTitle("Distributed Universe");

		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0, 1.0, 1.0, 7.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(1.6, 1.6, 1.6, 0.2, 0.0, 0.2, 0.0, 1.0, 0.0);

		glEnable(GL_DEPTH_TEST);

		double point_size = 3.0 * (100 / _num_blocks);
		if (point_size > 6) {
			point_size = 6;
		} else if (point_size < 2) {
			point_size = 2;
		}
		glPointSize(point_size);

		glClearColor(0.0, 0.0, 0.0, 1.0);

		_loop();
		return NULL;
	}

	void _loop() {
		_draw();
		pthread_mutex_lock(&_lock);
		while (true) {
			_wait_to_draw = true;
			while (_wait_to_draw) {
				pthread_cond_wait(&_can_draw, &_lock);
			}
			_draw();
		}
		pthread_mutex_unlock(&_lock);
	}

	void _draw() {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 1.0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);
		glEnd();

		glBegin(GL_POINTS);
		for (unsigned int y = 0; y < _num_blocks; ++y) {
			for (unsigned int x = 0; x < _num_blocks; ++x) {
				double x_value = ((double) x / _num_blocks);
				double y_value = ((double) y / _num_blocks);

				glColor3f((float) x / _num_blocks, (float) y / _num_blocks, 0.4f);
				double z_value;
				double divisor = _max_block_value - _min_block_value;
				double numerator = _block_values[y][x] - _min_block_value;
				if (divisor == 0 || _block_values[y][x] == 0) {
					z_value = 0;
				} else {
					z_value = numerator / divisor;
				}
				glVertex3f(x_value, z_value, y_value);
			}
		}
		glEnd();
		glFlush();
		glfwSwapBuffers();
		_max_block_value = 0;
		_min_block_value = _population;
	}

	void set_block_stat(uint32_t x, uint32_t y, uint32_t size) {
		pthread_mutex_lock(&_lock);
		_block_values[y][x] = size;
		if (size > _max_block_value) {
			_max_block_value = size;
		}
		if (size < _min_block_value) {
			_min_block_value = size;
		}
		_num_blocks_sets++;
		if (_num_blocks_sets == _num_blocks * _num_blocks) {
			_num_blocks_sets = 0;
			_wait_to_draw = false;
			pthread_cond_signal(&_can_draw);

		}
		pthread_mutex_unlock(&_lock);
	}
}
;
