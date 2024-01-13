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
    write("create com.cs1 empty transform rotation 0 0 0   translation 0 0 -0.055\n");
    write("create com.cs2 empty transform rotation 0 0 45  translation 0 0 -0.055\n");
    write("create com.cs3 empty transform rotation 0 0 90  translation 0 0 -0.055\n");
    write("create com.cs4 empty transform rotation 0 0 135 translation 0 0 -0.055\n");
    write("create com.cs5 empty transform rotation 0 0 180 translation 0 0 -0.055\n");
    write("create com.cs6 empty transform rotation 0 0 225 translation 0 0 -0.055\n");
    write("create com.cs7 empty transform rotation 0 0 270 translation 0 0 -0.055\n");
    write("create com.cs8 empty transform rotation 0 0 315 translation 0 0 -0.055\n");

    write("create com.body       model transform                 translation 0    0     -0.05  material color 0   0   255 path " + path + "/body.stl\n");
    write("create com.body.rotor model transform                 translation 0    0      0.101 material color 255 0   0   path " + path + "/rotor.stl\n");
    write("create com.cs1.fin1   model transform                 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs2.fin2   model transform rotation -5 0 0 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs3.fin3   model transform                 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs4.fin4   model transform rotation -5 0 0 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs5.fin5   model transform                 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs6.fin6   model transform rotation -5 0 0 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs7.fin7   model transform                 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");
    write("create com.cs8.fin8   model transform rotation -5 0 0 translation 0.03 0.006  0     material color 0   255 255 path " + path + "/fin.stl\n");

    write("create column1 cylinder transform rotation 90 0 0 translation  2 0 1 scale 0.1 2 0.1 material color 255 127 0\n");
    write("create column2 cylinder transform rotation 90 0 0 translation -2 0 1 scale 0.1 2 0.1 material color 255 127 0\n");

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
    const double x = state.q(0);
    const double y = state.q(1);
    const double z = state.q(2);
    const double phi = rad2deg*state.q(3);
    const double theta = rad2deg*state.q(4);
    const double psi = rad2deg*state.q(5);

    write(QString("update com transform translation %1 %2 %3\n").arg(x).arg(y).arg(z));
    write(QString("update com transform rotation    %1 %2 %3\n").arg(phi).arg(theta).arg(psi));
}
