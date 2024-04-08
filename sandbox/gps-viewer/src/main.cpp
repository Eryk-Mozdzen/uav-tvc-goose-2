#include <QApplication>

#include "Map.h"
#include "protocol/protocol_data.h"
#include "communication/Serial.h"

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	Map map;
	map.setView(51.1035, 17.0853);
	map.show();

	shared::Serial serial;

    QObject::connect(&serial, &shared::Serial::receive, [&](const protocol_message_t &message) {
		if(message.id!=PROTOCOL_ID_READINGS) {
			return;
		}

		const protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);

		if(readings->valid.gps) {
			map.append(readings->gps[0], readings->gps[1], "red");
		}
    });

	return app.exec();
}
