#ifndef CONNECTION_HANDLER_H_
#define CONNECTION_HANDLER_H_

#include <string>
#include <inttypes.h>

/**
 * An abstract class that provides an efficient buffer for receiving and handling socket messages
 *
 */
class ConnectionHandler {

	protected:
		int fd;

		//Bring a message (in part or whole) into the buffer and process. 1: Success, 0: Socket closed, -1: Error
		int fill_buffer_and_process();

		//Handles the message appropriately
		virtual void handle_message(unsigned char *message, uint32_t length) = 0;

	private:
		//Receive buffer (12 MiB)
		const static int BUFFER_SIZE = 12582912;
		unsigned char recv_buffer[BUFFER_SIZE];
		size_t unprocessed_bytes;

		//Attempts to process a message
		void process_buffer();

	public:

		ConnectionHandler(int fd);

		virtual ~ConnectionHandler();

		int get_fd();
};

#endif /* CONNECTION_HANDLER_H_ */
