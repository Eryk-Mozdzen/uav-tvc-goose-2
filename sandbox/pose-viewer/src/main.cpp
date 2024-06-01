#include <QCoreApplication>

#include "protocol/protocol_data.h"
#include "communication/Serial.h"
#include "communication/Telnet.h"
#include "communication/Visualization3d.h"

#define REF_LATITUDE    0.950871f // 54*28'51.2''
#define REF_LONGITUDE   0.323817f // 18*33'12.1''
#define EARTH_RADIUS    6371000.f
#define DEG_2_RAD       0.01745329251f

float cameraPosition[3] = {0, 0, 0};

void gps_to_enu(const float *position, float *cartesian) {
    const float lat = position[0]*DEG_2_RAD;
    const float lon = position[1]*DEG_2_RAD;

    const float ecef[3] = {
        EARTH_RADIUS*cosf(lat)*cosf(lon),
        EARTH_RADIUS*cosf(lat)*sinf(lon),
        EARTH_RADIUS*sinf(lat)
    };

    const float ecef_ref[3] = {
        EARTH_RADIUS*cosf(REF_LATITUDE)*cosf(REF_LONGITUDE),
        EARTH_RADIUS*cosf(REF_LATITUDE)*sinf(REF_LONGITUDE),
        EARTH_RADIUS*sinf(REF_LATITUDE)
    };

    const float s_phi = sinf(REF_LATITUDE);
    const float c_phi = cosf(REF_LATITUDE);
    const float s_lambda = sinf(REF_LONGITUDE);
    const float c_lambda = cosf(REF_LONGITUDE);

    const float R[3*3] = {
        -s_lambda, c_lambda, 0,
        -s_phi*c_lambda, -s_phi*s_lambda, c_phi,
        c_phi*c_lambda, c_phi*s_lambda, s_phi
    };

    cartesian[0] = R[0]*(ecef[0] - ecef_ref[0]) + R[1]*(ecef[1] - ecef_ref[1]) + R[2]*(ecef[2] - ecef_ref[2]);
    cartesian[1] = R[3]*(ecef[0] - ecef_ref[0]) + R[4]*(ecef[1] - ecef_ref[1]) + R[5]*(ecef[2] - ecef_ref[2]);
}

void normalize(const float *in, float *out, int dim) {
	float len2 = 0;

	for(int i=0; i<dim; i++) {
		len2 +=in[i]*in[i];
	}

	const float len = std::sqrt(len2);

	for(int i=0; i<dim; i++) {
		out[i] = in[i]/len;
	}
}

void receive(shared::Visualization3d &client, const protocol_message_t &message) {
	switch(message.id) {
		case PROTOCOL_ID_READINGS: {
			const protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);

			if(readings->valid.accelerometer) {
				float accelerometer[3];
				normalize(readings->calibrated.accelerometer, accelerometer, 3);
				client.write("update pos.marker.acc transform translation %f %f %f\n", accelerometer[0], accelerometer[1], accelerometer[2]);
			}

			if(readings->valid.magnetometer) {
				float magnetometer[3];
				normalize(readings->calibrated.magnetometer, magnetometer, 3);
				client.write("update pos.marker.mag transform translation %f %f %f\n", magnetometer[0], magnetometer[1], magnetometer[2]);
			}

			if(readings->valid.gps) {
				float cartesian[2];
				gps_to_enu(readings->gps, cartesian);
				client.write("update gps transform translation %f %f 0\n", cartesian[0], cartesian[1]);
			} else {
				client.write("update gps transform translation 0 0 0\n");
			}
		} break;
		case PROTOCOL_ID_ESTIMATION: {
			const protocol_estimation_t *estimation = reinterpret_cast<protocol_estimation_t *>(message.payload);

			constexpr float alpha = 0.99f;
			cameraPosition[0] = alpha*cameraPosition[0] + (1 - alpha)*estimation->position[0];
			cameraPosition[1] = alpha*cameraPosition[1] + (1 - alpha)*estimation->position[1];
			cameraPosition[2] = alpha*cameraPosition[2] + (1 - alpha)*estimation->position[2];
			client.write("camera %f %f %f\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);

			float quaternion[4];
			normalize(estimation->orientation, quaternion, 4);
			client.write("update pos.marker transform quaternion %f %f %f %f\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

			client.write("update pos transform translation %f %f %f\n",
				estimation->position[0],
				estimation->position[1],
				estimation->position[2]
			);

			client.write("update pos.vel transform translation %f %f %f\n",
				estimation->velocity[0],
				estimation->velocity[1],
				estimation->velocity[2]
			);
		} break;
	}
}

int main(int argc, char *argv[]) {
	QCoreApplication app(argc, argv);

	shared::Serial serial;
	shared::Telnet telnet;
	shared::Visualization3d client;

	if(argc>1) {
		client.write("mode %s\n", argv[1]);
	}

	client.write("clear\n");
	client.write("create pos            empty\n");
	client.write("create pos.marker     cuboid material color 255 255 255 geometry 0.25 0.25 0.25\n");
	client.write("create pos.marker.x   cuboid material color 255   0   0 geometry 1.00 0.05 0.05 transform translation 0.5 0 0\n");
	client.write("create pos.marker.y   cuboid material color   0 255   0 geometry 0.05 1.00 0.05 transform translation 0 0.5 0\n");
	client.write("create pos.marker.z   cuboid material color   0   0 255 geometry 0.05 0.05 1.00 transform translation 0 0 0.5\n");
	client.write("create pos.marker.acc sphere material color   0 255   0 geometry 0.05\n");
	client.write("create pos.marker.mag sphere material color   0   0 255 geometry 0.05\n");
	client.write("create pos.vel        sphere material color 255   0   0 geometry 0.05\n");
	client.write("create gps            sphere material color 255   0 255 geometry 0.15\n");

	QObject::connect(&serial, &shared::Serial::receive, std::bind(receive, std::ref(client), std::placeholders::_1));
	QObject::connect(&telnet, &shared::Telnet::receive, std::bind(receive, std::ref(client), std::placeholders::_1));

	return app.exec();
}
