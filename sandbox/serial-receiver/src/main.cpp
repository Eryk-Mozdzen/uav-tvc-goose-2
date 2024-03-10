#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>

#include "protocol.h"

int main() {

	int port = open("/dev/ttyACM0", O_RDWR);
	if(port<0) {
		std::cerr << "can't open port" << std::endl;
		return 1;
	}

	uint8_t buffer[64];
	uint8_t id;
	uint8_t payload[1024];
	uint16_t size;

	while(true) {
		const int n = read(port, buffer, sizeof(buffer));

		for(int i=0; i<n; i++) {
			if(protocol_decode(buffer[i], &id, payload, &size)) {
				std::cout << std::string(reinterpret_cast<char *>(payload), size);
			}

			if(buffer[i]==0) {
				std::cout << std::endl;
			}
		}
	}

	close(port);
}
