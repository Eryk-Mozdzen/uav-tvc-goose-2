#include <iostream>
#include <iomanip>

#include <QCoreApplication>

#include "protocol/protocol_data.h"
#include "communication/Serial.h"
#include "communication/Telnet.h"

constexpr float rad2deg = 57.2957795131f;

std::ostream & operator<<(std::ostream &stream, const protocol_readings_t readings) {
	stream << "press";
	stream << std::setprecision(0) << std::fixed << std::noshowpos << std::setw(7);
	stream << (readings.valid.barometer ? readings.barometer : std::nan(""));

	stream << "   range";
	stream << std::setprecision(2) << std::fixed << std::noshowpos << std::setw(5);
	stream << (readings.valid.rangefinder ? readings.rangefinder : std::nan(""));

	stream << "   mag [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << (readings.valid.magnetometer ? readings.calibrated.magnetometer[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.magnetometer ? readings.calibrated.magnetometer[1] : std::nan(""));
	stream << std::setw(6) << (readings.valid.magnetometer ? readings.calibrated.magnetometer[2] : std::nan(""));
	stream << "]";

	stream << "   accel [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << (readings.valid.accelerometer ? readings.calibrated.accelerometer[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.accelerometer ? readings.calibrated.accelerometer[1] : std::nan(""));
	stream << std::setw(6) << (readings.valid.accelerometer ? readings.calibrated.accelerometer[2] : std::nan(""));
	stream << "]";

	stream << "   gyro [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << (readings.valid.gyroscope ? readings.calibrated.gyroscope[0] : std::nan(""));
	stream << std::setw(6) << (readings.valid.gyroscope ? readings.calibrated.gyroscope[1] : std::nan(""));
	stream << std::setw(6) << (readings.valid.gyroscope ? readings.calibrated.gyroscope[2] : std::nan(""));
	stream << "]";

	stream << "   gps [";
	stream << std::setprecision(6) << std::fixed << std::noshowpos;
	stream << std::setw(10) << (readings.valid.gps ? readings.gps[0] : std::nan(""));
	stream << std::setw(10) << (readings.valid.gps ? readings.gps[1] : std::nan(""));
	stream << "]";

	return stream;
}

std::ostream & operator<<(std::ostream &stream, const protocol_estimation_t estimation) {
	stream << "qua [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << estimation.orientation[0];
	stream << std::setw(6) << estimation.orientation[1];
	stream << std::setw(6) << estimation.orientation[2];
	stream << std::setw(6) << estimation.orientation[3];
	stream << "]";

	stream << "   omega [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << estimation.angular_velocity[0];
	stream << std::setw(6) << estimation.angular_velocity[1];
	stream << std::setw(6) << estimation.angular_velocity[2];
	stream << "]";

	stream << "   pos [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << estimation.position[0];
	stream << std::setw(6) << estimation.position[1];
	stream << std::setw(6) << estimation.position[2];
	stream << "]";

	stream << "   vel [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << estimation.velocity[0];
	stream << std::setw(6) << estimation.velocity[1];
	stream << std::setw(6) << estimation.velocity[2];
	stream << "]";

	stream << "   accel [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << estimation.acceleration[0];
	stream << std::setw(6) << estimation.acceleration[1];
	stream << std::setw(6) << estimation.acceleration[2];
	stream << "]";

	return stream;
}

void receive(const int filter, const protocol_message_t &message) {
	if(!(filter & (1<<message.id))) {
		return;
	}

	switch(message.id) {
		case PROTOCOL_ID_LOG: {
			const char *str = reinterpret_cast<char *>(message.payload);
			std::cout << std::string(str, message.size) << std::endl;
		} break;
		case PROTOCOL_ID_READINGS: {
			const protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);
			std::cout << *readings << std::endl;
		} break;
		case PROTOCOL_ID_PASSTHROUGH_GPS: {
			const char *str = reinterpret_cast<char *>(message.payload);
			std::cout << std::string(str, message.size);
		} break;
		case PROTOCOL_ID_ESTIMATION: {
			const protocol_estimation_t *estimation = reinterpret_cast<protocol_estimation_t *>(message.payload);
			std::cout << *estimation << std::endl;
		} break;
		default: {
			std::cout << "unknown message" << std::endl;
		} break;
	}
}

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);

	int filter = 0x00;
	for(int i=1; i<argc; i++) {
		const int id = QString::fromUtf8(argv[i]).toInt();
		filter |=(1<<id);
	}

	shared::Serial serial;
	shared::Telnet telnet;

	QObject::connect(&serial, &shared::Serial::receive, std::bind(receive, filter, std::placeholders::_1));
	QObject::connect(&telnet, &shared::Telnet::receive, std::bind(receive, filter, std::placeholders::_1));

	return app.exec();
}
