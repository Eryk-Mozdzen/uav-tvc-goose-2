#include <QCoreApplication>

#include "protocol/protocol_data.h"
#include "communication/Serial.h"
#include "communication/Telnet.h"
#include "communication/Visualization3d.h"

void normalize(const float *in, float *out) {
	const float len = std::sqrt(in[0]*in[0] + in[1]*in[1] + in[2]*in[2]);
	out[0] = in[0]/len;
	out[1] = in[1]/len;
	out[2] = in[2]/len;
}

void receive(shared::Visualization3d &client, const protocol_message_t &message) {
	switch(message.id) {
		case PROTOCOL_ID_READINGS: {
			const protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);

			if(readings->valid.accelerometer) {
				float norm[3];
				normalize(readings->calibrated.accelerometer, norm);
				client.write("update goose.acc transform translation %f %f %f\n", norm[0], norm[1], norm[2]);
			}

			if(readings->valid.magnetometer) {
				float norm[3];
				normalize(readings->calibrated.magnetometer, norm);
				client.write("update goose.mag transform translation %f %f %f\n", norm[0], norm[1], norm[2]);
			}
		} break;
		case PROTOCOL_ID_ESTIMATION: {
			const protocol_estimation_t *estimation = reinterpret_cast<protocol_estimation_t *>(message.payload);

			const float len = std::sqrt(
				estimation->orientation[0]*estimation->orientation[0] +
				estimation->orientation[1]*estimation->orientation[1] +
				estimation->orientation[2]*estimation->orientation[2] +
				estimation->orientation[3]*estimation->orientation[3]
			);

			client.write("update goose transform quaternion %f %f %f %f translation %f %f %f\n",
				estimation->orientation[0]/len,
				estimation->orientation[1]/len,
				estimation->orientation[2]/len,
				estimation->orientation[3]/len,
				estimation->position[0],
				estimation->position[1],
				estimation->position[2]
			);
		} break;
	}
}

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);

	shared::Serial serial;
	shared::Telnet telnet;
	shared::Visualization3d client;

	client.write("clear\n");
	client.write("create goose            empty\n");
	client.write("create goose.acc        empty\n");
	client.write("create goose.mag        empty\n");
	client.write("create goose.marker     cuboid material color 255 255 255 transform scale 0.25 0.25 0.25\n");
	client.write("create goose.markerX    cuboid material color 255   0   0 transform scale 1.00 0.05 0.05 translation 0.5 0 0\n");
	client.write("create goose.markerY    cuboid material color   0 255   0 transform scale 0.05 1.00 0.05 translation 0 0.5 0\n");
	client.write("create goose.markerZ    cuboid material color   0   0 255 transform scale 0.05 0.05 1.00 translation 0 0 0.5\n");
	client.write("create goose.acc.marker sphere material color 0   255   0 transform scale 0.05 0.05 0.05\n");
	client.write("create goose.mag.marker sphere material color 0   0   255 transform scale 0.05 0.05 0.05\n");

	QObject::connect(&serial, &shared::Serial::receive, std::bind(receive, std::ref(client), std::placeholders::_1));
	QObject::connect(&telnet, &shared::Telnet::receive, std::bind(receive, std::ref(client), std::placeholders::_1));

	return app.exec();
}
