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
  m_slider = new RobotGui::CustomizedSlider(Qt::Orientation::Horizontal);
  m_slider->setMaximumWidth(120);
  m_slider->setMinimum(lower);
  m_slider->setMaximum(upper);
  lay->addWidget(m_slider, line_number, 1);

  m_valueofjoint = new QLabel;
  m_valueofjoint->setText(QString::number(0));
  m_valueofjoint->setFixedWidth(80);
  lay->addWidget(m_valueofjoint, line_number, 2);

  QObject::connect(m_slider, SIGNAL(valueChanged(int)),
                   signalmapper, SLOT(map()));
  signalmapper->setMapping(m_slider, slider_id);
  QObject::connect(signalmapper, SIGNAL(mapped(int)),
                   recevier, SLOT(sliderPositionChanged(int)));
}

const float JointSliderWidget::getSliderPosition() const {
    return m_slider->value();
}

const QString JointSliderWidget::getJointName() const {
    return m_joint_name;
}

void JointSliderWidget::updateSliderRange(float lowerBound, float upperBound) {
  m_slider->setMinimum(lowerBound);
  m_slider->setMaximum(upperBound);
}

void JointSliderWidget::set_labelvalue() {
  float settingValue = getSliderPosition();
  m_valueofjoint->setText(QString::number(settingValue, 'f', 1) + tr(" deg"));
}

void JointSliderWidget::setSliderPosition(const float value) {
  double setvalue = value;
  m_slider->setValue(setvalue,true);
  set_labelvalue();
  m_slider->blockSignals(false);
}

#include "moc_WidgetUtility.cpp"
