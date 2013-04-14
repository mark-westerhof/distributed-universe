#ifndef NETUTILS_H_
#define NETUTILS_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <string>

namespace netutils {

	const uint32_t IP_ADDRESS_LENGTH = INET6_ADDRSTRLEN;
	const uint32_t BDP_BUFFER_SIZE = 3276800000;

	// Gets the textual IP address from a sockaddr_storage
	void get_ip_textual(sockaddr_storage *sas, char* ip_address);

	// Gets the textual IP address from a addrinfo
	void get_ip_textual(addrinfo *ai, char* ip_address);

	// Inserts an integer into the specified location of a message after converting to network byte order
	void insert_uint32_into_message(uint32_t value, unsigned char *location);

	// Returns an integer from the specified location of a message after converting to host byte order
	uint32_t get_uint32_from_message(unsigned char *location);

	// Wrapper around send to do our best to send the entire message. Returns 0: success, -1: error
	int sendall(int socket, unsigned char *message, int len);

	// Wrapper around recv to do our best to receive the entire message. Returns # bytes received, 0 if closed, -1 on error
	int recvall(int socket, unsigned char *message, int len);

	// Disables the Nagle algorithm for a TCP socket connection (TCP_NODELAY). Returns 0: success, -1: error
	int disable_nagle_algorithm(int socket);

	// Sets the BDP buffer size for a TCP socket connection. Returns 0: success, -1: error
	int set_bdp(int socket);

}

#endif /* NETUTILS_H_ */
