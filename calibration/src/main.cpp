#include <QApplication>

#include "communication/Serial.h"
#include "Window.h"

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	shared::Serial serial;

	Window window;
	window.show();

	QObject::connect(&serial, &shared::Serial::receive, &window, &Window::receive);
	QObject::connect(&window, &Window::transmit, &serial, &shared::Serial::transmit);

	return app.exec();
}
