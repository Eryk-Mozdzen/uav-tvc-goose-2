#include <QCoreApplication>

#include "protocol/protocol_data.h"
#include "communication/Serial.h"
#include "communication/Telnet.h"
#include "communication/Visualization3d.h"

void receive(shared::Visualization3d &client, const protocol_message_t &message) {
	switch(message.id) {
		case PROTOCOL_ID_READINGS: {
			const protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);

			if(readings->valid.accelerometer) {
				client.write("update goose.accel transform translation %f %f %f\n", readings->calibrated.accelerometer[0], readings->calibrated.accelerometer[1], readings->calibrated.accelerometer[2]);
			}

			if(readings->valid.magnetometer) {
				client.write("update goose.mag transform translation %f %f %f\n", readings->calibrated.magnetometer[0], readings->calibrated.magnetometer[1], readings->calibrated.magnetometer[2]);
			}
		} break;
	}
}

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);

	shared::Serial serial;
	shared::Telnet telnet;
	shared::Visualization3d client;

	client.write("clear\n");
	client.write("create goose        sphere   material color 255 255 255 transform scale 0.1 0.1 0.1\n");
	client.write("create goose.accel  sphere   material color 0   255 0  \n");
	client.write("create goose.mag    sphere   material color 0   0   255\n");

	QObject::connect(&serial, &shared::Serial::receive, std::bind(receive, std::ref(client), std::placeholders::_1));
	QObject::connect(&telnet, &shared::Telnet::receive, std::bind(receive, std::ref(client), std::placeholders::_1));

	return app.exec();
}
