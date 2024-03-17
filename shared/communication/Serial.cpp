#include <stdexcept>

#include <QSerialPort>

#include "Serial.h"
#include "protocol/protocol.h"

namespace shared {

Serial::Serial(const char *port, QObject *parent) : QObject{parent} {
    decoder.buffer = rx_buffer;
    decoder.size = sizeof(rx_buffer);
    decoder.counter = 0;

    serial.setPortName(port);
    serial.setBaudRate(115200);

    if(!serial.open(QIODevice::OpenModeFlag::ReadWrite)) {
        throw std::runtime_error("can't open port");
	}

    connect(&serial, &QSerialPort::readyRead, [&]() {
        const QByteArray data = serial.readAll();

		for(const uint8_t byte : data) {
			protocol_message_t message;

			if(protocol_decode(&decoder, byte, &message)) {
				receive(message);
			}
		}
    });
}

void Serial::transmit(const protocol_message_t &message) {
    const uint16_t size = protocol_encode(tx_buffer, &message);

	serial.write(reinterpret_cast<char *>(tx_buffer), size);
    serial.waitForBytesWritten(100);
}

}
