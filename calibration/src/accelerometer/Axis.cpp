#include <QGroupBox>
#include <QGridLayout>
#include <QLineEdit>
#include <QPushButton>

#include "Axis.h"

Axis::Axis(const char name, QWidget *parent) : QWidget{parent} {
	QGridLayout *one = new QGridLayout(this);

	QGroupBox *group = new QGroupBox(QString(name) + " axis");

	one->addWidget(group, 0, 0);

	line_sensor = new QLineEdit();
	button_pos = new QPushButton("sample");
	button_neg = new QPushButton("sample");
	line_pos = new QLineEdit();
	line_neg = new QLineEdit();
	line_offset = new QLineEdit();
	line_scale = new QLineEdit();

	line_sensor->setReadOnly(true);
	line_sensor->setPlaceholderText("max positive");
	line_sensor->setAlignment(Qt::AlignHCenter);
	line_pos->setReadOnly(true);
	line_pos->setPlaceholderText("max positive");
	line_pos->setAlignment(Qt::AlignHCenter);
	line_pos->setFixedWidth(100);
	line_neg->setReadOnly(true);
	line_neg->setPlaceholderText("max negative");
	line_neg->setAlignment(Qt::AlignHCenter);
	line_neg->setFixedWidth(100);
	line_offset->setReadOnly(true);
	line_offset->setPlaceholderText("offset");
	line_offset->setAlignment(Qt::AlignHCenter);
	line_scale->setReadOnly(true);
	line_scale->setPlaceholderText("scale");
	line_scale->setAlignment(Qt::AlignHCenter);

	QGridLayout *grid = new QGridLayout(group);
	grid->addWidget(line_sensor, 0, 0, 1, 2);
	grid->addWidget(button_pos, 1, 0);
	grid->addWidget(button_neg, 2, 0);
	grid->addWidget(line_pos, 1, 1);
	grid->addWidget(line_neg, 2, 1);
	grid->addWidget(line_scale, 3, 0, 1, 2);
	grid->addWidget(line_offset, 4, 0, 1, 2);

	connect(button_pos, &QPushButton::clicked, [&]() {
		const float pos = line_sensor->text().toFloat();

		line_pos->setText(QString::asprintf("%5.4f", pos));

		if(line_neg->text().size()>0) {
			const float neg = line_neg->text().toFloat();

			update(neg, pos);

			line_offset->setText(QString::asprintf("%5.4f", offset));
			line_scale->setText(QString::asprintf("%5.4f", scale));
		}
	});

	connect(button_neg, &QPushButton::clicked, [&]() {
		const float neg = line_sensor->text().toFloat();

		line_neg->setText(QString::asprintf("%5.4f", neg));

		if(line_pos->text().size()>0) {
			const float pos = line_pos->text().toFloat();

			update(neg, pos);

			line_offset->setText(QString::asprintf("%5.4f", offset));
			line_scale->setText(QString::asprintf("%5.4f", scale));
		}
	});

}

void Axis::update(const double min, const double max) {
	constexpr double g = 9.8065;

	scale = 2*g/(max - min);
	offset = -g*(max + min)/(max - min);
}

void Axis::set(const float sensor) {
    line_sensor->setText(QString::asprintf("%5.4f", sensor));
}

double Axis::getScale() const {
	return scale;
}

double Axis::getOffset() const {
	return offset;
}
