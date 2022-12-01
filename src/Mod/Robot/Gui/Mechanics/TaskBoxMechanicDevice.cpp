#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <QPushButton>
#include <QFileDialog>
#include <Base/Console.h>
#include <Gui/BitmapFactory.h>
#include "TaskBoxMechanicDevice.h".h"
#include "Mod/Robot/App/Utilites/FileIO_Utility.h"
#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxMechanicDevice.h"

using namespace RobotGui;

TaskBoxPositionerSetupPanel::TaskBoxPositionerSetupPanel(Robot::MechanicDevice *t_Positioner)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"), tr("Setup Panel")) {
  if (t_Positioner == nullptr)
    return;
  m_DocPtr = t_Positioner->getDocument();
  m_Positioner = t_Positioner;
  initUi_PanelWidgets();
}

void TaskBoxPositionerSetupPanel::initUi_PanelWidgets() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxMechanicDevice();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  m_JointPanel_layout = new QGridLayout;

  QObject::connect(m_ui->pushButton_setHomePose, SIGNAL(clicked()),
                   this, SLOT(slot_setCurrentPoseAsHomePosition()));
  QObject::connect(m_ui->pushButton_toHomePose, SIGNAL(clicked()),
                   this, SLOT(slot_setPositionerToHomePose()));
  QObject::connect(m_ui->pushButton_importCalibData,SIGNAL(clicked()),
                   this, SLOT(slot_importCalibData()));

  initUi_AxisControllerBox();
  initUi_ConfigurationBox();
  initUi_PoseSetupBox();
}

void TaskBoxPositionerSetupPanel::initUi_AxisControllerBox() {
  if (m_JointPanel_layout == nullptr)
    return;

  if(m_signalmapper == nullptr)
     m_signalmapper = new QSignalMapper(this);

  m_jointSliderVec.clear();

  for (int jntID = 0; jntID<m_Positioner->getJointNumbers(); jntID++)
  {
    auto jointName = std::string("J-")+std::to_string(jntID+1);
    auto slider_widget = new JointSliderWidget(m_ui->groupBox_AxisControl,
                                               m_JointPanel_layout,
                                               jntID+1,jntID,
                                               QString::fromStdString(jointName),
                                               this,
                                               m_signalmapper,
                                               m_Positioner->getJointMaxAngle(jntID),
                                               m_Positioner->getJointMinAngle(jntID));
    m_jointSliderVec.push_back(slider_widget);
  }

  slot_updateSliderPosition();
}

void TaskBoxPositionerSetupPanel::initUi_ConfigurationBox()
{
    for(int i = 1; i<=m_Positioner->getJointNumbers(); i++){
        QString jnt_Name = QObject::tr("Jnt-")+QString::number(i);
        m_ui->comboBox_JointList->addItem(jnt_Name);
    }
    m_ui->doubleSpinBox_lowerLimit->setValue(m_Positioner->getJointMinAngle(0));
    m_ui->doubleSpinBox_lowerLimit->setMinimum(m_Positioner->getJointMinAngle(0));
    m_ui->doubleSpinBox_lowerLimit->setMaximum(m_Positioner->getJointMaxAngle(0));

    m_ui->doubleSpinBox_upperLimit->setValue(m_Positioner->getJointMaxAngle(0));
    m_ui->doubleSpinBox_upperLimit->setMinimum(m_Positioner->getJointMinAngle(0));
    m_ui->doubleSpinBox_upperLimit->setMaximum(m_Positioner->getJointMaxAngle(0));

    QObject::connect(m_ui->comboBox_JointList, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_targetConfigJointChanged()));
    QObject::connect(m_ui->pushButton_updateJointConfig, SIGNAL(clicked(bool)),
                     this, SLOT(slot_changeTargetJointLimits()));
    QObject::connect(m_ui->radioButton_rotDir_norm, SIGNAL(clicked(bool)),
                     this, SLOT(slot_flipTargetAxisDirection()));
    QObject::connect(m_ui->radioButton_rotDir_invrt, SIGNAL(clicked(bool)),
                     this, SLOT(slot_flipTargetAxisDirection()));
    slot_targetConfigJointChanged();
}

void TaskBoxPositionerSetupPanel::initUi_PoseSetupBox()
{
    m_ui->comboBox_Reference->clear();
    m_ui->comboBox_Reference->addItem(tr("World"));
    auto robotList = m_DocPtr->getObjectsOfType(Robot::Robot6AxisObject::getClassTypeId());
    for(auto rbtObject : robotList){
        m_ui->comboBox_Reference->addItem(tr(rbtObject->getNameInDocument()));
    }

    slot_updatePositionerPosePanel();

    QObject::connect(m_ui->doubleSpinBox_tX, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePositionerPose()));
    QObject::connect(m_ui->doubleSpinBox_tY, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePositionerPose()));
    QObject::connect(m_ui->doubleSpinBox_tZ, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePositionerPose()));
    QObject::connect(m_ui->doubleSpinBox_rX, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePositionerPose()));
    QObject::connect(m_ui->doubleSpinBox_rY, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePositionerPose()));
    QObject::connect(m_ui->doubleSpinBox_rZ, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_updatePositionerPose()));
    QObject::connect(m_ui->comboBox_Reference,SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_referenceTargetChanged()));
    QObject::connect(m_ui->pushButton_BindReference, SIGNAL(clicked(bool)),
                     this, SLOT(slot_updatePositionerReference()));
}

void TaskBoxPositionerSetupPanel::slot_updatePositionerPosePanel()
{
    auto c_Trans = m_Positioner->Pose_Ref2Base.getValue();
    m_ui->doubleSpinBox_tX->setValue(c_Trans.getPosition().x);
    m_ui->doubleSpinBox_tY->setValue(c_Trans.getPosition().y);
    m_ui->doubleSpinBox_tZ->setValue(c_Trans.getPosition().z);
    auto c_Rotate = c_Trans.getRotation();
    double y,p,r;
    c_Rotate.getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_rZ->setValue(y);
    m_ui->doubleSpinBox_rY->setValue(p);
    m_ui->doubleSpinBox_rX->setValue(r);

    auto c_RefNameStr = m_Positioner->OriginReference.getStrValue();
    for(int i = 0; i<m_ui->comboBox_Reference->count();i++){
        if(c_RefNameStr == m_ui->comboBox_Reference->itemText(i).toStdString()){
            m_ui->comboBox_Reference->setCurrentIndex(i);
            m_ui->pushButton_BindReference->setEnabled(false);
        }
    }
}

void TaskBoxPositionerSetupPanel::slot_targetConfigJointChanged()
{
    auto jntID = m_ui->comboBox_JointList->currentIndex();
  //  updateJointLimitationPanel(jntID);
    m_ui->doubleSpinBox_lowerLimit->blockSignals(true);
    m_ui->doubleSpinBox_upperLimit->blockSignals(true);
    m_ui->doubleSpinBox_lowerLimit->setMinimum(m_Positioner->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_lowerLimit->setMaximum(m_Positioner->getJointMaxAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setMinimum(m_Positioner->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setMaximum(m_Positioner->getJointMaxAngle(jntID));
    m_ui->doubleSpinBox_lowerLimit->setValue(m_Positioner->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setValue(m_Positioner->getJointMaxAngle(jntID));
    m_ui->radioButton_rotDir_norm->setChecked(!m_Positioner->isAxisDirInverted(jntID));
    m_ui->radioButton_rotDir_invrt->setChecked(m_Positioner->isAxisDirInverted(jntID));
    m_ui->doubleSpinBox_lowerLimit->blockSignals(false);
    m_ui->doubleSpinBox_upperLimit->blockSignals(false);
}

void TaskBoxPositionerSetupPanel::slot_changeTargetJointLimits()
{
    auto jntID = m_ui->comboBox_JointList->currentIndex();
    auto lowerLimits = m_Positioner->LowerLimits_Soft.getValues();
    lowerLimits[jntID] = m_ui->doubleSpinBox_lowerLimit->value();
    auto upperLimits = m_Positioner->UpperLimits_Soft.getValues();
    upperLimits[jntID] = m_ui->doubleSpinBox_upperLimit->value();
    m_Positioner->LowerLimits_Soft.setValues(lowerLimits);
    m_Positioner->UpperLimits_Soft.setValues(upperLimits);
}

void TaskBoxPositionerSetupPanel::slot_importCalibData()
{
    QDialogCalibrationPanel* t_CalibDlg = new QDialogCalibrationPanel(m_Positioner);
    QObject::connect(t_CalibDlg, SIGNAL(signal_updatePosePanel()),
                     this, SLOT(slot_updatePositionerPosePanel()));
    t_CalibDlg->show();
}

void TaskBoxPositionerSetupPanel::slot_flipTargetAxisDirection()
{
    auto axis_ID = m_ui->comboBox_JointList->currentIndex();
    m_Positioner->flipAxisDirection(axis_ID, m_ui->radioButton_rotDir_invrt->isChecked());
}

void TaskBoxPositionerSetupPanel::slot_updateSliderPosition() {
  for (int jntID = 0; jntID < m_Positioner->getJointNumbers(); jntID++) {
    auto sliderPtr = m_jointSliderVec[jntID];
    sliderPtr->setSliderPosition(m_Positioner->getJointAngle(jntID));
  }
}


void TaskBoxPositionerSetupPanel::slot_setPositionerToHomePose()
{
    m_Positioner->restoreHomePose();
}



void TaskBoxPositionerSetupPanel::slot_setCurrentPoseAsHomePosition() {
    m_Positioner->setCurrentPoseAsHome();
}

void TaskBoxPositionerSetupPanel::slot_updatePositionerPose()
{
    Base::Vector3d t_Trans;
    Base::Rotation t_Rot;
    Base::Placement new_Trans;
    t_Trans.x = m_ui->doubleSpinBox_tX->value();
    t_Trans.y = m_ui->doubleSpinBox_tY->value();
    t_Trans.z = m_ui->doubleSpinBox_tZ->value();
    t_Rot.setYawPitchRoll(m_ui->doubleSpinBox_rZ->value(),
                          m_ui->doubleSpinBox_rY->value(),
                          m_ui->doubleSpinBox_rX->value());
    new_Trans = Base::Placement(t_Trans,t_Rot);
    m_Positioner->Pose_Ref2Base.setValue(new_Trans);
}

void TaskBoxPositionerSetupPanel::slot_updatePositionerReference()
{
    if(m_ui->comboBox_Reference->currentIndex()==0){
        m_Positioner->Pose_Reference.setValue(Base::Placement());
        m_Positioner->OriginReference.setValue("World");
    }
    else{
        auto t_NameOfRefRBT = m_ui->comboBox_Reference->currentText().toStdString();
        auto t_RefRbtPtr = static_cast<Robot::Robot6AxisObject*>(m_DocPtr->getObject(t_NameOfRefRBT.c_str()));
        if(t_RefRbtPtr!=nullptr){
            m_Positioner->Pose_Reference.setValue(t_RefRbtPtr->getCurrentBasePose());
            m_Positioner->OriginReference.setValue(t_NameOfRefRBT.c_str());
        }
    }
    m_ui->pushButton_BindReference->setEnabled(false);
}

void TaskBoxPositionerSetupPanel::slot_referenceTargetChanged()
{
    auto c_selected = m_ui->comboBox_Reference->currentText().toStdString();
    m_ui->pushButton_BindReference->setEnabled(c_selected != m_Positioner->OriginReference.getStrValue());
}

void TaskBoxPositionerSetupPanel::sliderPositionChanged(int t_Index) {
  if (m_jointSliderVec.empty())
    return;
  if (t_Index < m_jointSliderVec.size()) {
    auto sliderPtr = m_jointSliderVec[t_Index];
    m_Positioner->setJointAngle(t_Index, sliderPtr->getSliderPosition());
    sliderPtr->set_labelvalue();
  }
}

JointSliderWidget *TaskBoxPositionerSetupPanel::getTargetJointSlider(const int jntID) {
    if(jntID<0 || jntID>m_jointSliderVec.size())
        return nullptr;
    return m_jointSliderVec.at(jntID);
}


bool TaskBoxPositionerSetupPanel::accept() {
    m_Positioner->Activated.setValue(false);
    return true;
}

bool TaskBoxPositionerSetupPanel::reject() {
    m_Positioner->Activated.setValue(false);
    return true;
}



#include "moc_TaskBoxMechanicDevice.cpp";
