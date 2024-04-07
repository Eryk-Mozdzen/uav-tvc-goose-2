#include <QApplication>
#include <QDebug>
#include <QTime>

#include "Map.h"
#include "protocol/protocol_data.h"
#include "communication/Serial.h"

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	shared::Serial serial;

	Map map(51.103525733902586, 17.085345490751223);
	map.show();

    QObject::connect(&serial, &shared::Serial::receive, [&](const protocol_message_t &message) {
		if(message.id!=PROTOCOL_ID_READINGS) {
			return;
		}

		const protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(message.payload);

		if(!readings->valid.gps) {
			qDebug() << QTime::currentTime().toString() << "GPS data not valid";
			return;
		}

        map.mark(readings->gps[0], readings->gps[1]);
    });

	return app.exec();
}
