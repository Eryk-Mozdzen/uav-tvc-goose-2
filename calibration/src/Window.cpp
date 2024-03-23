#include <sstream>
#include <iomanip>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTextEdit>
#include <Eigen/Dense>

#include "Window.h"
#include "Magnetometer.h"

std::ostream & operator<<(std::ostream &stream, const protocol_calibration_t &calibration) {
    const Eigen::Matrix3d mag_A {
        {calibration.magnetometer[0], calibration.magnetometer[1], calibration.magnetometer[2]},
        {calibration.magnetometer[3], calibration.magnetometer[4], calibration.magnetometer[5]},
        {calibration.magnetometer[6], calibration.magnetometer[7], calibration.magnetometer[8]}
    };

    const Eigen::Vector3d mag_b {
        calibration.magnetometer[9],
        calibration.magnetometer[10],
        calibration.magnetometer[11]
    };

    const Eigen::Matrix3d accel_A = Eigen::DiagonalMatrix<double, 3, 3>({
        calibration.accelerometer[0],
        calibration.accelerometer[1],
        calibration.accelerometer[2]
    });

    const Eigen::Vector3d accel_b {
        calibration.accelerometer[3],
        calibration.accelerometer[4],
        calibration.accelerometer[5]
    };

    stream << std::setprecision(4) << std::fixed << std::showpos;
    stream << "magnetometer scale:\n" << mag_A << "\n";
    stream << "magnetometer offset:\n" << mag_b << "\n";
    stream << "accelerometer scale:\n" << accel_A << "\n";
    stream << "accelerometer offset:\n" << accel_b << "\n";

    return stream;
}

Window::Window(QWidget *parent) : QWidget{parent}, current{nullptr} {
    memset(&calibration, 0, sizeof(calibration));

    interfaces.push_back(new Magnetometer());

    QGridLayout *grid = new QGridLayout(this);

    {
        QGroupBox *group = new QGroupBox("applications");
        QHBoxLayout *layout  = new QHBoxLayout(group);

        group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

        for(Interface *interface : interfaces) {
            QPushButton *button = new QPushButton(interface->getName(), group);

            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

            connect(button, &QPushButton::clicked, std::bind(&Window::setCurrent, this, interface));

            layout->addWidget(button);
        }

        grid->addWidget(group, 0, 0);
    }

    {
        QGroupBox *group = new QGroupBox("interface");
        interface_layout = new QHBoxLayout(group);

        group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        grid->addWidget(group, 1, 0);
    }

    {
        QGroupBox *group = new QGroupBox("parameters");
        QVBoxLayout *layout  = new QVBoxLayout(group);

        group->setFixedWidth(250);

        calibration_text = new QTextEdit(group);
        calibration_text->setReadOnly(true);
        QPushButton *button_read = new QPushButton("read from device", group);
        QPushButton *button_update = new QPushButton("update parameters", group);
        QPushButton *button_set = new QPushButton("write to device", group);

        connect(button_read, &QPushButton::clicked, [&]() {
            calibration_text->setText("waiting for data...");
            const protocol_message_t calibration_request = {nullptr, 0, PROTOCOL_ID_CALIBRATION};
            transmit(calibration_request);
        });

        connect(button_update, &QPushButton::clicked, [&]() {
            if(current) {
                current->update(calibration);

                std::ostringstream stream;
                stream << calibration;
                calibration_text->setText(QString::fromStdString(stream.str()));
            } else {
                calibration_text->setText("app not selected");
            }
        });

        connect(button_set, &QPushButton::clicked, [&]() {
            const protocol_message_t calibration_frame = {
                &calibration,
                sizeof(calibration),
                PROTOCOL_ID_CALIBRATION
            };
            transmit(calibration_frame);
        });

        layout->addWidget(calibration_text);
        layout->addWidget(button_read);
        layout->addWidget(button_update);
        layout->addWidget(button_set);

        grid->addWidget(group, 0, 1, 2, 1);
    }
}

void Window::setCurrent(Interface *interface) {
    if(current) {
        interface_layout->removeWidget(current);
    }
    current = interface;
    interface_layout->addWidget(current);
}

void Window::receive(const protocol_message_t &frame) {
    if(current && frame.id==PROTOCOL_ID_READINGS) {
        protocol_readings_t *readings = reinterpret_cast<protocol_readings_t *>(frame.payload);

        current->receive(*readings);

        update();

        return;
    }

    if(frame.id==PROTOCOL_ID_CALIBRATION) {
        memcpy(&calibration, frame.payload, sizeof(calibration));

        std::ostringstream stream;
        stream << calibration;
        calibration_text->setText(QString::fromStdString(stream.str()));
    }
}
