#pragma once

#include <vector>

#include <QWidget>
#include <QTextEdit>
#include <QHBoxLayout>

#include "protocol/protocol.h"
#include "protocol/protocol_data.h"
#include "Interface.h"

class Window : public QWidget {
    Q_OBJECT

    protocol_calibration_t calibration;
    std::vector<Interface *> interfaces;
    Interface *current;
    QTextEdit *calibration_text;
    QHBoxLayout *interface_layout;

    void setCurrent(Interface *interface);

signals:
    void transmit(const protocol_message_t &frame);

public slots:
    void receive(const protocol_message_t &frame);

public:
    Window(QWidget *parent = nullptr);
};
