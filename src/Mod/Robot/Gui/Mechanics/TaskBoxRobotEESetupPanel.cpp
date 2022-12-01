#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBoxRobotEESetupPanel.h"

#include <Base/Console.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <QPushButton>
#include <Mod/Robot/Gui/Mechanics/ViewProviderRobot6AxisObject.h>
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxRobotEESetupPanel.h"

using namespace RobotGui;

TaskBoxRobotEESetupPanel::TaskBoxRobotEESetupPanel(Robot::Robot6AxisObject *t_robot,
                                                   Gui::TaskView::TaskSelectLinkProperty *t_FaceSelection,
                                                   Gui::TaskView::TaskSelectLinkProperty *t_EdgeSelection)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),tr("Tip Pose Setup")) {
  if (t_robot == nullptr || t_EdgeSelection == nullptr)
    return;
  m_DocPtr = t_robot->getDocument();
  m_targetRobot = t_robot;
  m_FaceRef = t_FaceSelection;
  m_EdgeRef = t_EdgeSelection;

  if (!initUi())
      return;
}

void TaskBoxRobotEESetupPanel::mouseReleaseEvent(QMouseEvent *ev)
{
}

bool TaskBoxRobotEESetupPanel::accept() {
//    if(t_D)
//    t_Dragger->destroyDragger();
    return true;
}

bool TaskBoxRobotEESetupPanel::reject() {
//    t_Dragger->destroyDragger();
    return true;
}

bool TaskBoxRobotEESetupPanel::initUi() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxRobotEESetupPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  m_ui->label_TargetRobotName->setText(tr(m_targetRobot->getNameInDocument()));

  QObject::connect(m_ui->pushButton_CalcCenter, SIGNAL(clicked()),
                   this, SLOT(slot_getSelectedCenter()));
  QObject::connect(m_ui->pushButton_Assemble, SIGNAL(clicked()),
                   this, SLOT(slot_assembleTool2Robot()));
  QObject::connect(m_ui->pushButton_finishSetup, SIGNAL(clicked()),
                   this, SLOT(slot_finishSetup()));

  initUi_TorchBox();
  initUi_SensorBox();
  return true;
}

bool TaskBoxRobotEESetupPanel::initUi_TorchBox()
{
    QObject::connect(m_ui->doubleSpinBox_Torch_tX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateTorchPose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_tY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateTorchPose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_tZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateTorchPose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_rX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateTorchPose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_rY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateTorchPose()));
    QObject::connect(m_ui->doubleSpinBox_Torch_rZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateTorchPose()));
    QObject::connect(m_ui->comboBox_TorchSetup_target, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_currentSettingTargetChanged()));

    updatePanel_TorchPanel();
}

bool TaskBoxRobotEESetupPanel::initUi_SensorBox()
{
    QObject::connect(m_ui->doubleSpinBox_Sensor_tX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateSensorPose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_tY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateSensorPose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_tZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateSensorPose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_rX, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateSensorPose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_rY, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateSensorPose()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_rZ, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_updateSensorPose()));
    QObject::connect(m_ui->comboBox_SensorSetup_target, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_currentSettingTargetChanged()));

    QObject::connect(m_ui->pushButton_LaserOn, SIGNAL(clicked()),
                     this, SLOT(slot_setLaserOn()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_distance, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeSensorViz()));
    QObject::connect(m_ui->doubleSpinBox_Sensor_amplitude, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeSensorViz()));

    updatePanel_SensorPanel();
}

void TaskBoxRobotEESetupPanel::blockTorchPosePanelSignals(bool block)
{
    m_ui->doubleSpinBox_Torch_tX->blockSignals(block);
    m_ui->doubleSpinBox_Torch_tY->blockSignals(block);
    m_ui->doubleSpinBox_Torch_tZ->blockSignals(block);
    m_ui->doubleSpinBox_Torch_rX->blockSignals(block);
    m_ui->doubleSpinBox_Torch_rY->blockSignals(block);
    m_ui->doubleSpinBox_Torch_rZ->blockSignals(block);
}

void TaskBoxRobotEESetupPanel::blockSensorPosePanelSignals(bool block)
{
    m_ui->doubleSpinBox_Sensor_tX->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_tY->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_tZ->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_rX->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_rY->blockSignals(block);
    m_ui->doubleSpinBox_Sensor_rZ->blockSignals(block);
}

void TaskBoxRobotEESetupPanel::updatePanel_TorchPanel()
{
    Base::Placement c_Pose;
    auto t_Target = m_ui->comboBox_TorchSetup_target->currentText();
    if(t_Target == QObject::tr("Tip")){
        c_Pose = m_targetRobot->Trans_Flan2TorchTip.getValue();
    }
    else if(t_Target == QObject::tr("Base")){
        c_Pose = m_targetRobot->Trans_Flan2TorchBase.getValue();
    }
    blockTorchPosePanelSignals(true);

    m_ui->doubleSpinBox_Torch_tX->setValue(c_Pose.getPosition().x);
    m_ui->doubleSpinBox_Torch_tY->setValue(c_Pose.getPosition().y);
    m_ui->doubleSpinBox_Torch_tZ->setValue(c_Pose.getPosition().z);
    double y,p,r;
    c_Pose.getRotation().getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_Torch_rX->setValue(r);
    m_ui->doubleSpinBox_Torch_rY->setValue(p);
    m_ui->doubleSpinBox_Torch_rZ->setValue(y);

    blockTorchPosePanelSignals(false);
}

void TaskBoxRobotEESetupPanel::updatePanel_SensorPanel()
{

    Base::Placement c_Pose;
    auto t_Target = m_ui->comboBox_SensorSetup_target->currentText();
    if(t_Target == QObject::tr("CMOS")){
        c_Pose = m_targetRobot->Trans_Flan2SensorOrigin.getValue();
    }
    else if(t_Target == QObject::tr("Base")){
        c_Pose = m_targetRobot->Trans_Flan2SensorBase.getValue();
    }
    blockSensorPosePanelSignals(true);

    m_ui->doubleSpinBox_Sensor_tX->setValue(c_Pose.getPosition().x);
    m_ui->doubleSpinBox_Sensor_tY->setValue(c_Pose.getPosition().y);
    m_ui->doubleSpinBox_Sensor_tZ->setValue(c_Pose.getPosition().z);
    double y,p,r;
    c_Pose.getRotation().getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_Sensor_rX->setValue(r);
    m_ui->doubleSpinBox_Sensor_rY->setValue(p);
    m_ui->doubleSpinBox_Sensor_rZ->setValue(y);

    blockSensorPosePanelSignals(false);
}

void TaskBoxRobotEESetupPanel::updateDraggerPose(const Base::Placement& t_Pose)
{
    if(t_Dragger == nullptr){
        auto robotVP_Ptr = dynamic_cast<ViewProviderRobot6AxisObject*>(Gui::Application::Instance->activeDocument()->getViewProvider(m_targetRobot));
        t_Dragger = new InteractiveDragger(robotVP_Ptr->getRoot(),t_Pose);
    }
    t_Dragger->setDraggerPosition(t_Pose);
}

void TaskBoxRobotEESetupPanel::slot_getSelectedCenter()
{
    if(m_FaceRef==nullptr)
        return;

    if(m_FaceRef->isSelectionValid()){
        m_FaceRef->sendSelection2Property();
    }

    if(m_EdgeRef->isSelectionValid()){
        m_EdgeRef->sendSelection2Property();
    }

    Base::Placement selectedCenter = m_targetRobot->getSelectedFeatureCenter();

    switch(m_CurrentTarget){
    case SetTarget::TorchTip:
        m_targetRobot->Trans_Flan2TorchTip.setValue(selectedCenter);
        break;
    case SetTarget::TorchBase:
        m_targetRobot->Trans_Flan2TorchBase.setValue(selectedCenter);
        break;
    case SetTarget::SensorCMOS:
        m_targetRobot->Trans_Flan2SensorOrigin.setValue(selectedCenter);
        break;
    case SetTarget::SensorBase:
        m_targetRobot->Trans_Flan2SensorBase.setValue(selectedCenter);
        break;
    default:
        break;
    }

    updatePanel_TorchPanel();
    updatePanel_SensorPanel();
}

void TaskBoxRobotEESetupPanel::slot_assembleTool2Robot()
{
    // Get Selection

    m_targetRobot->Visiable.setValue(true);
}

void TaskBoxRobotEESetupPanel::slot_updateTorchPose()
{
    if (m_targetRobot == nullptr)
      return;
    Base::Placement newPose;
    newPose.setPosition(Base::Vector3d(m_ui->doubleSpinBox_Torch_tX->value(),
                                       m_ui->doubleSpinBox_Torch_tY->value(),
                                       m_ui->doubleSpinBox_Torch_tZ->value()));
    Base::Rotation newRot;
    newRot.setYawPitchRoll(m_ui->doubleSpinBox_Torch_rZ->value(),
                           m_ui->doubleSpinBox_Torch_rY->value(),
                           m_ui->doubleSpinBox_Torch_rX->value());
    newPose.setRotation(newRot);

    auto t_Target = m_ui->comboBox_TorchSetup_target->currentText();
    if(t_Target == QObject::tr("Tip")){
        m_targetRobot->Trans_Flan2TorchTip.setValue(newPose);
    }
    else if(t_Target == QObject::tr("Base")){
        m_targetRobot->Trans_Flan2TorchBase.setValue(newPose);
    }
}

void TaskBoxRobotEESetupPanel::slot_updateSensorPose()
{
    if (m_targetRobot == nullptr)
      return;
    Base::Placement newPose;
    newPose.setPosition(Base::Vector3d(m_ui->doubleSpinBox_Sensor_tX->value(),
                                       m_ui->doubleSpinBox_Sensor_tY->value(),
                                       m_ui->doubleSpinBox_Sensor_tZ->value()));
    Base::Rotation newRot;
    newRot.setYawPitchRoll(m_ui->doubleSpinBox_Sensor_rZ->value(),
                           m_ui->doubleSpinBox_Sensor_rY->value(),
                           m_ui->doubleSpinBox_Sensor_rX->value());
    newPose.setRotation(newRot);

    auto t_Target = m_ui->comboBox_SensorSetup_target->currentText();
    if(t_Target == QObject::tr("CMOS")){
        m_targetRobot->Trans_Flan2SensorOrigin.setValue(newPose);
    }
    else if(t_Target == QObject::tr("Base")){
        m_targetRobot->Trans_Flan2SensorBase.setValue(newPose);
    }
}

void TaskBoxRobotEESetupPanel::slot_currentSettingTargetChanged()
{
    auto c_PageIndex = m_ui->tabWidget_ToolSelection->currentIndex();
    if(c_PageIndex == 1){
        auto c_TorchIndex = m_ui->comboBox_TorchSetup_target->currentIndex();
        if(c_TorchIndex == 1)
            m_CurrentTarget == SetTarget::TorchTip;
        if(c_TorchIndex == 2)
            m_CurrentTarget == SetTarget::TorchBase;
    }
    else if(c_PageIndex == 2){
        auto c_SensorIndex = m_ui->comboBox_SensorSetup_target->currentIndex();
        if(c_SensorIndex == 1)
            m_CurrentTarget == SetTarget::SensorBase;
        if(c_SensorIndex == 2)
            m_CurrentTarget == SetTarget::SensorCMOS;
    }
    updatePanel_TorchPanel();
    updatePanel_SensorPanel();
}

void TaskBoxRobotEESetupPanel::slot_setLaserOn()
{
//    if(m_targetRobot->SensorOn.getValue()){
//        m_targetRobot->SensorOn.setValue(true);
//        m_ui->pushButton_LaserOn->setText(tr("Turn Off"));
//    }
//    else{
//        m_targetRobot->SensorOn.setValue(false);
//        m_ui->pushButton_LaserOn->setText(tr("Turn On"));
//    }
}

void TaskBoxRobotEESetupPanel::slot_changeSensorViz()
{
//    m_targetRobot->SensorDistance.setValue(m_ui->doubleSpinBox_Sensor_distance->value());
//    m_targetRobot->SensorAmplitude.setValue(m_ui->doubleSpinBox_Sensor_amplitude->value());
}

void TaskBoxRobotEESetupPanel::slot_finishSetup()
{
    Q_EMIT Signal_finishSetup();
}


#include "Mechanics/moc_TaskBoxRobotEESetupPanel.cpp"
