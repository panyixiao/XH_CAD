#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "WidgetUtility.h"

using namespace RobotGui;

CustomizedSlider::CustomizedSlider(Qt::Orientation orientation, QWidget* pParent) :
    QSlider(orientation,pParent),
    m_Multiplier(10000.0)
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
    setSingleStep(1);
    setOrientation(Qt::Horizontal);
    setFocusPolicy(Qt::NoFocus);
}

void CustomizedSlider::setValue(int Value)
{
    Q_EMIT valueChanged((double)Value / m_Multiplier);
}

void CustomizedSlider::setValue(double Value, bool BlockSignals)
{
    QSlider::blockSignals(BlockSignals);
    QSlider::setValue(Value * m_Multiplier);
    if (!BlockSignals)
        Q_EMIT valueChanged(Value);

    QSlider::blockSignals(false);
}

void CustomizedSlider::setRange(double Min, double Max)
{
    QSlider::setRange(Min * m_Multiplier, Max * m_Multiplier);

    Q_EMIT rangeChanged(Min, Max);
}

void CustomizedSlider::setMinimum(double Min)
{
    QSlider::setMinimum(Min * m_Multiplier);

    Q_EMIT rangeChanged(minimum(), maximum());
}

double CustomizedSlider::minimum() const
{
    return QSlider::minimum() / m_Multiplier;
}

void CustomizedSlider::setMaximum(double Max)
{
    QSlider::setMaximum(Max * m_Multiplier);

    Q_EMIT rangeChanged(minimum(), maximum());
}

double CustomizedSlider::maximum() const
{
    return QSlider::maximum() / m_Multiplier;
}

double CustomizedSlider::value() const
{
    int Value = QSlider::value();
    return (double)Value / m_Multiplier;
}

/////*****************************************************************
/////
/////
/////
/////*****************************************************************

JointSliderWidget::JointSliderWidget(QWidget *parent,
                                     QGridLayout *lay,
                                     uint line_number,
                                     uint slider_id,
                                     QString jointname,
                                     QObject *recevier,
                                     QSignalMapper *signalmapper,
                                     float upper, float lower): QDialog(parent)
{
  parent->setLayout(lay);
  m_nameofjoint = new QLabel;
  m_joint_name = jointname;
  m_nameofjoint->setText(jointname);
  m_nameofjoint->setMaximumSize(80, 15);
  m_nameofjoint->setAlignment(Qt::AlignHCenter);
  lay->addWidget(m_nameofjoint, line_number, 0);
  m_slider_JntVal = new RobotGui::CustomizedSlider(Qt::Orientation::Horizontal);
  m_slider_JntVal->setMaximumWidth(120);
  lay->addWidget(m_slider_JntVal, line_number, 1);

  m_spinBox_JntVal = new QDoubleSpinBox;
  m_spinBox_JntVal->setValue(0.0);
  m_spinBox_JntVal->setDecimals(2);
  m_spinBox_JntVal->setFixedWidth(80);
  lay->addWidget(m_spinBox_JntVal, line_number, 2);

  QObject::connect(m_slider_JntVal, SIGNAL(valueChanged(int)),
                   signalmapper, SLOT(map()));
  QObject::connect(m_slider_JntVal, SIGNAL(valueChanged(int)),
                   this, SLOT(slot_updateJntValSpinBoxBySliderPos()));
  signalmapper->setMapping(m_slider_JntVal, slider_id);
  QObject::connect(signalmapper, SIGNAL(mapped(int)),
                   recevier, SLOT(slot_sliderPositionChanged(int)));

  QObject::connect(m_spinBox_JntVal, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_updateSliderPosByJntValSpinBox()));

  updateSliderRange(lower,upper);
}

float JointSliderWidget::getSliderPosition() const {
    return m_slider_JntVal->value();
}

QString JointSliderWidget::getJointName() const {
    return m_joint_name;
}

void JointSliderWidget::updateSliderRange(float lowerBound, float upperBound) {
  m_slider_JntVal->setMinimum(lowerBound);
  m_slider_JntVal->setMaximum(upperBound);
  m_spinBox_JntVal->setMaximum(upperBound);
  m_spinBox_JntVal->setMinimum(lowerBound);
}

void JointSliderWidget::updateAxisWidgetData(const float value) {
//  double setvalue = value;
//  m_slider_JntVal->setValue(setvalue,true);
//  m_slider_JntVal->blockSignals(false);
  m_spinBox_JntVal->setValue(value);
}

void JointSliderWidget::slot_updateSliderPosByJntValSpinBox()
{
    m_slider_JntVal->blockSignals(true);
    m_slider_JntVal->setValue(m_spinBox_JntVal->value());
    m_slider_JntVal->blockSignals(false);
}

void JointSliderWidget::slot_updateJntValSpinBoxBySliderPos()
{
    m_spinBox_JntVal->blockSignals(true);
    m_spinBox_JntVal->setValue(getSliderPosition());
    m_spinBox_JntVal->blockSignals(false);
}

#include "moc_WidgetUtility.cpp"
