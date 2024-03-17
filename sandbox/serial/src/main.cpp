#include <iostream>
#include <iomanip>

#include <QCoreApplication>
#include <QSerialPort>
#include <QTimer>

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

std::ostream & operator<<(std::ostream &stream, const protocol_estimation_t estimation) {
	stream << "qua [";
	stream << std::setprecision(2) << std::fixed << std::showpos;
	stream << std::setw(6) << estimation.orientation[0];
	stream << std::setw(6) << estimation.orientation[1];
	stream << std::setw(6) << estimation.orientation[2];
	stream << std::setw(6) << estimation.orientation[3];
	stream << "]";

	return stream;
}

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);

	QSerialPort serial;
	serial.setPortName("/dev/ttyACM0");
	serial.setBaudRate(115200);

	if(!serial.open(QIODevice::OpenModeFlag::ReadWrite)) {
		std::cerr << "can't open port" << std::endl;
		return 1;
	}

	uint8_t buffer[1024];

	protocol_decoder_t decoder {
		buffer,
		sizeof(buffer),
		0
	};

	QTimer timer;

	QObject::connect(&timer, &QTimer::timeout, [&]() {
		const QByteArray data = serial.readAll();

		for(const uint8_t byte : data) {
			protocol_message_t message;
			if(!protocol_decode(&decoder, byte, &message)) {
				continue;
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
				case PROTOCOL_ID_ESTIMATION: {
					const protocol_estimation_t *estimation = reinterpret_cast<protocol_estimation_t *>(message.payload);
					std::cout << *estimation << std::endl;
				} break;
				default: {
					std::cout << "unknown message" << std::endl;
				} break;
			}
		}
	});

	timer.start(10);

	return app.exec();
}
