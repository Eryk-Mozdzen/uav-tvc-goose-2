#pragma once

#include <QLineEdit>
#include <QPushButton>

class Axis : public QWidget {
	double scale = 1;
	double offset = 0;

	QLineEdit *line_sensor;
	QPushButton *button_pos;
	QPushButton *button_neg;
	QLineEdit *line_pos;
	QLineEdit *line_neg;
	QLineEdit *line_offset;
	QLineEdit *line_scale;

	void update(const double min, const double max);

public:
    Axis(const char name, QWidget *parent=nullptr);

    void set(const float sensor);
	double getScale() const;
	double getOffset() const;
};
