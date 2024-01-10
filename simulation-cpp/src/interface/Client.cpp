#include <sstream>

#include <QHostAddress>
#include <QFileInfo>

#include "Client.h"

Client::Client() {
    socket.connectToHost(QHostAddress::LocalHost, 8080);
    if(!socket.waitForConnected()) {
        qDebug() << "server not found";
        return;
    }

    const QString path = QFileInfo(".").canonicalPath() + "/models";

    write("clear\n");

    write("create com empty\n");
    write("create com.cs1 empty transform rotation 0 0 0.00000 translation 0 0 -0.055\n");
    write("create com.cs2 empty transform rotation 0 0 0.78539 translation 0 0 -0.055\n");
    write("create com.cs3 empty transform rotation 0 0 1.57079 translation 0 0 -0.055\n");
    write("create com.cs4 empty transform rotation 0 0 2.35619 translation 0 0 -0.055\n");
    write("create com.cs5 empty transform rotation 0 0 3.14159 translation 0 0 -0.055\n");
    write("create com.cs6 empty transform rotation 0 0 3.92699 translation 0 0 -0.055\n");
    write("create com.cs7 empty transform rotation 0 0 4.71238 translation 0 0 -0.055\n");
    write("create com.cs8 empty transform rotation 0 0 5.49778 translation 0 0 -0.055\n");

    write("create com.body       model transform translation 0 0 -0.05  material color 0 0 255 path " + path + "/body.stl\n");
    write("create com.body.rotor model transform translation 0 0  0.101 material color 255 0 0 path " + path + "/rotor.stl\n");
    write("create com.cs1.fin1   model transform                        translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs2.fin2   model transform rotation -0.087266 0 0 translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs3.fin3   model transform                        translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs4.fin4   model transform rotation -0.087266 0 0 translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs5.fin5   model transform                        translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs6.fin6   model transform rotation -0.087266 0 0 translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs7.fin7   model transform                        translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");
    write("create com.cs8.fin8   model transform rotation -0.087266 0 0 translation 0.03 0.006 0 material color 0 255 255 path " + path + "/fin.stl\n");

    write("create col1 cylinder transform rotation 1.57079 0 0 translation  2 0 1 scale 0.1 2 0.1 material color 255 127 0\n");
    write("create col2 cylinder transform rotation 1.57079 0 0 translation -2 0 1 scale 0.1 2 0.1 material color 255 127 0\n");

    connect(&socket, &QTcpSocket::disconnected, this, &Client::disconnect);
}

void Client::write(const QString &message) {
    if(socket.state()==QAbstractSocket::UnconnectedState) {
        disconnect();
        return;
    }

    socket.write(message.toUtf8());

    if(!socket.waitForBytesWritten()) {
        disconnect();
    }
}

void Client::draw(const Object::State &state) {

    const QString command = QString("update com transform rotation %1 %2 %3 translation %4 %5 %6\n").arg(state.q(3)).arg(state.q(4)).arg(state.q(5)).arg(state.q(0)).arg(state.q(1)).arg(state.q(2));

    write(command);
}
