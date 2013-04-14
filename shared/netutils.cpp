#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>

#include "netutils.h"

void netutils::get_ip_textual(sockaddr_storage *sas, char* ip_address) {
	if (sas->ss_family == AF_INET) {
		inet_ntop(sas->ss_family, &((sockaddr_in*) sas)->sin_addr, ip_address, netutils::IP_ADDRESS_LENGTH);
	} else {
		inet_ntop(sas->ss_family, &((sockaddr_in6*) sas)->sin6_addr, ip_address, netutils::IP_ADDRESS_LENGTH);
	}
}

void netutils::get_ip_textual(addrinfo *ai, char* ip_address) {
	if (ai->ai_family == AF_INET) {
		inet_ntop(ai->ai_family, &((sockaddr_in*) ai->ai_addr)->sin_addr, ip_address, netutils::IP_ADDRESS_LENGTH);
	} else {
		inet_ntop(ai->ai_family, &((sockaddr_in6*) ai->ai_addr)->sin6_addr, ip_address, netutils::IP_ADDRESS_LENGTH);
	}
}

void netutils::insert_uint32_into_message(uint32_t value, unsigned char *location) {
	*((uint32_t *) location) = htonl(value);
}

uint32_t netutils::get_uint32_from_message(unsigned char *location) {
	return ntohl(*((uint32_t*) location));
}

int netutils::sendall(int socket, unsigned char *message, int len) {

	int total_bytes_sent = 0;
	int bytes_remaining = len;

	while (total_bytes_sent < len) {
		int bytes_sent = send(socket, message + total_bytes_sent, bytes_remaining, 0);
		if (bytes_sent < 0) {
			return -1;
		}
		total_bytes_sent += bytes_sent;
		bytes_remaining -= bytes_sent;
	}
	return 0;
}

int netutils::recvall(int socket, unsigned char *message, int len) {

	int total_bytes_recieved = 0;
	int bytes_remaining = len;

	while (total_bytes_recieved < len) {
		int bytes_received = recv(socket, message + total_bytes_recieved, bytes_remaining, 0);
		if (bytes_received <= 0) {
			return bytes_received;
		}
		total_bytes_recieved += bytes_received;
		bytes_remaining -= bytes_received;
	}
	return total_bytes_recieved;
}

int netutils::disable_nagle_algorithm(int socket) {
	int flag = 1;
	return setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
}

int netutils::set_bdp(int socket) {
	int return_val;
	return_val = setsockopt(socket, SOL_SOCKET, SO_SNDBUF, (char *) &netutils::BDP_BUFFER_SIZE,
			sizeof(netutils::BDP_BUFFER_SIZE));
	if (return_val < 0) {
		return return_val;
	}
	return setsockopt(socket, SOL_SOCKET, SO_RCVBUF, (char *) &netutils::BDP_BUFFER_SIZE,
			sizeof(netutils::BDP_BUFFER_SIZE));
}
