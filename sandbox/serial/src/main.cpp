#include <iostream>
#include <iomanip>
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

	uint8_t serial[64];
	uint8_t buffer[1024];
	uint8_t payload[1024];

	protocol_decoder_t decoder {
		buffer,
		sizeof(buffer),
		0
	};

	protocol_message_t message {
		payload,
		0,
		0
	};

	while(true) {
		const int n = read(port, serial, sizeof(serial));

		for(int i=0; i<n; i++) {
			if(protocol_decode(&decoder, serial[i], &message)) {
				std::cout << std::string(reinterpret_cast<char *>(message.payload), message.size);
			}

			//std::cout << std::right << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (int)serial[i] << " ";

			if(serial[i]==0) {
				std::cout << std::endl;
			}
		}
	}

	close(port);
}
