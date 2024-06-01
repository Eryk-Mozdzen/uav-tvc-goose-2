#pragma once

#include <QTcpSocket>

namespace shared {

class Visualization3d {
    QTcpSocket socket;

public:
	Visualization3d();
	~Visualization3d();

	void write(const char *format, ...);
};

}
