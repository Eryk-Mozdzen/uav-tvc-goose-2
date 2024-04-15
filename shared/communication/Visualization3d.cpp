#include <cstdarg>

#include <QTcpSocket>

#include "Visualization3d.h"

namespace shared {

Visualization3d::Visualization3d() {
    socket.connectToHost("localhost", 8080);
    socket.waitForConnected();
}

Visualization3d::~Visualization3d() {
    socket.close();
}

void Visualization3d::write(const char *format, ...) {
    va_list args;
    va_start(args, format);

	char str[256];
    const size_t len = vsprintf(str, format, args);

    socket.write(str, len);
    socket.flush();
    socket.waitForBytesWritten();
}

}
