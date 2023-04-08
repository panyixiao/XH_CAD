// Created by Yixiao 2022-08-09
#ifndef ROBOTGUI_UTILITY_WIDGET_H
#define ROBOTGUI_UTILITY_WIDGET_H

#include <QObject>
#include <QDialog>
#include <QWidget>
#include <QGridLayout>
#include <QPlainTextEdit>
#include <QSignalMapper>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>

namespace RobotGui {

class CustomizedSlider : public QSlider
{
    Q_OBJECT
public:
    CustomizedSlider(Qt::Orientation orientation, QWidget* pParent = NULL);

    void setRange(double Min, double Max);
    void setMinimum(double Min);
    double minimum() const;
    void setMaximum(double Max);
    double maximum() const;
    double value() const;

public Q_SLOTS:
    void setValue(int value);
    void setValue(double Value, bool BlockSignals = false);

Q_SIGNALS:
    void valueChanged(double Value);
    void rangeChanged(double Min, double Max);

private:
    double	m_Multiplier;
};

/////*****************************************************************
/////
/////
/////
/////*****************************************************************

class JointSliderWidget : public QDialog {
  Q_OBJECT

public:
  JointSliderWidget(QWidget *parent,
                    QGridLayout *lay,
                    uint line_number,
                    uint slider_id,
                    QString jointname,
                    QObject *recevier,
                    QSignalMapper *signalmapper,
                    float upper, float lower);
  float getSliderPosition() const;
  QString getJointName() const;
  void updateSliderRange(float lowerBound, float upperBound);
  void updateAxisWidgetData(const float);

protected Q_SLOTS:
  void slot_updateSliderPosByJntValSpinBox();
  void slot_updateJntValSpinBoxBySliderPos();

private:
  int value;
  QString m_joint_name;
  CustomizedSlider *m_slider_JntVal;
  QDoubleSpinBox *m_spinBox_JntVal;
  QLabel *m_nameofjoint;
};

}

#endif
