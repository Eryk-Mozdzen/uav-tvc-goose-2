#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>

#include "protocol.h"
#include "protocol_data.h"

std::ostream & operator<<(std::ostream &stream, const protocol_readings_t readings) {
	stream << "press";
	stream << std::setprecision(0) << std::fixed << std::noshowpos << std::setw(7);
	stream << (readings.valid.barometer ? readings.barometer : std::nan(""));

	stream << "   range";
	stream << std::setprecision(2) << std::fixed << std::noshowpos << std::setw(5);
	stream << (readings.valid.rangefinder ? readings.rangefinder : std::nan(""));

	stream << "   mag [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << (readings.valid.magnetometer ? readings.magnetometer[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.magnetometer ? readings.magnetometer[1] : std::nan(""));
	stream << std::setw(6) << (readings.valid.magnetometer ? readings.magnetometer[2] : std::nan(""));
	stream << "]";

	stream << "   accel [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << (readings.valid.accelerometer ? readings.accelerometer[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.accelerometer ? readings.accelerometer[1] : std::nan(""));
	stream << std::setw(6) << (readings.valid.accelerometer ? readings.accelerometer[2] : std::nan(""));
	stream << "]";

	stream << "   gyro [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << (readings.valid.gyroscope ? readings.gyroscope[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.gyroscope ? readings.gyroscope[1] : std::nan(""));
	stream << std::setw(6) << (readings.valid.gyroscope ? readings.gyroscope[2] : std::nan(""));
	stream << "]";

	stream << "   gps [";
	stream << std::setprecision(2) << std::fixed << std::noshowpos;
	stream << std::setw(6) << (readings.valid.gps ? readings.gps[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.gps ? readings.gps[1] : std::nan(""));
	stream << "]";

	return stream;
}

int main() {

	int port = open("/dev/ttyACM0", O_RDONLY);
	if(port<0) {
		std::cerr << "can't open port" << std::endl;
		return 1;
	}

	uint8_t serial[256];
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
				switch(message.id) {
					case PROTOCOL_ID_LOG: {
						std::cout << std::string(reinterpret_cast<char *>(message.payload), message.size) << std::endl;
					} break;
					case PROTOCOL_ID_READINGS: {
						protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);
						std::cout << *readings << std::endl;
					} break;
					default: {
						std::cout << "unknown id" << std::endl;
					} break;
				}
			}
		}
	}

	close(port);
}
