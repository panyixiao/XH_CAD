#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <QPushButton>
#include <QFileDialog>
#include <Base/Console.h>
#include <Gui/BitmapFactory.h>
#include "TaskBoxExtAxSetupPanel.h"
#include "Mod/Robot/App/Utilites/FileIO_Utility.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxExtAxSetupPanel.h"

using namespace RobotGui;

TaskBoxExtAxSetupPanel::TaskBoxExtAxSetupPanel(Robot::MechanicExtAx *t_ExtAxisPtr)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"), tr("Setup Panel")) {
  if (t_ExtAxisPtr == nullptr)
    return;
  m_DocPtr = t_ExtAxisPtr->getDocument();
  m_ExtAxisPtr = t_ExtAxisPtr;
  initUi_PanelWidgets();
}

void TaskBoxExtAxSetupPanel::initUi_PanelWidgets() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxExtAxSetupPanel();
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

void TaskBoxExtAxSetupPanel::initUi_AxisControllerBox() {
  if (m_JointPanel_layout == nullptr)
    return;

  if(m_signalmapper == nullptr)
     m_signalmapper = new QSignalMapper(this);

  m_jointSliderVec.clear();

  for (size_t jntID = 0; jntID<m_ExtAxisPtr->getJointNumbers(); jntID++)
  {
    auto jointName = std::string("J-")+std::to_string(jntID+1);
    auto slider_widget = new JointSliderWidget(m_ui->groupBox_AxisControl,
                                               m_JointPanel_layout,
                                               jntID+1,jntID,
                                               QString::fromStdString(jointName),
                                               this,
                                               m_signalmapper,
                                               m_ExtAxisPtr->getJointMaxAngle(jntID),
                                               m_ExtAxisPtr->getJointMinAngle(jntID));
    m_jointSliderVec.push_back(slider_widget);
  }

  slot_updateSliderPosition();
}

void TaskBoxExtAxSetupPanel::initUi_ConfigurationBox()
{
    for(size_t i = 1; i<=m_ExtAxisPtr->getJointNumbers(); i++){
        QString jnt_Name = QObject::tr("Jnt-")+QString::number(i);
        m_ui->comboBox_JointList->addItem(jnt_Name);
    }
    m_ui->doubleSpinBox_lowerLimit->setValue(m_ExtAxisPtr->getJointMinAngle(0));
    m_ui->doubleSpinBox_lowerLimit->setMinimum(m_ExtAxisPtr->getJointMinAngle(0));
    m_ui->doubleSpinBox_lowerLimit->setMaximum(m_ExtAxisPtr->getJointMaxAngle(0));

    m_ui->doubleSpinBox_upperLimit->setValue(m_ExtAxisPtr->getJointMaxAngle(0));
    m_ui->doubleSpinBox_upperLimit->setMinimum(m_ExtAxisPtr->getJointMinAngle(0));
    m_ui->doubleSpinBox_upperLimit->setMaximum(m_ExtAxisPtr->getJointMaxAngle(0));

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

void TaskBoxExtAxSetupPanel::initUi_PoseSetupBox()
{
    m_ui->comboBox_Reference->clear();
    m_ui->comboBox_Reference->addItem(tr("World"));
//    auto robotList = m_DocPtr->getObjectsOfType(Robot::Mech::getClassTypeId());
//    for(auto rbtObject : robotList){
//        m_ui->comboBox_Reference->addItem(tr(rbtObject->getNameInDocument()));
//    }

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
//    QObject::connect(m_ui->pushButton_BindReference, SIGNAL(clicked(bool)),
//                     this, SLOT(slot_updatePositionerReference()));
}

void TaskBoxExtAxSetupPanel::slot_updatePositionerPosePanel()
{
    auto c_Trans = m_ExtAxisPtr->Trans_Ref2Base.getValue();
    m_ui->doubleSpinBox_tX->setValue(c_Trans.getPosition().x);
    m_ui->doubleSpinBox_tY->setValue(c_Trans.getPosition().y);
    m_ui->doubleSpinBox_tZ->setValue(c_Trans.getPosition().z);
    auto c_Rotate = c_Trans.getRotation();
    double y,p,r;
    c_Rotate.getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_rZ->setValue(y);
    m_ui->doubleSpinBox_rY->setValue(p);
    m_ui->doubleSpinBox_rX->setValue(r);

//    auto c_RefNameStr = m_Positioner->OriginReference.getStrValue();
//    for(int i = 0; i<m_ui->comboBox_Reference->count();i++){
//        if(c_RefNameStr == m_ui->comboBox_Reference->itemText(i).toStdString()){
//            m_ui->comboBox_Reference->setCurrentIndex(i);
//            m_ui->pushButton_BindReference->setEnabled(false);
//        }
//    }
}

void TaskBoxExtAxSetupPanel::slot_targetConfigJointChanged()
{
    auto jntID = m_ui->comboBox_JointList->currentIndex();
    m_ui->doubleSpinBox_lowerLimit->blockSignals(true);
    m_ui->doubleSpinBox_upperLimit->blockSignals(true);
    m_ui->doubleSpinBox_lowerLimit->setMinimum(m_ExtAxisPtr->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_lowerLimit->setMaximum(m_ExtAxisPtr->getJointMaxAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setMinimum(m_ExtAxisPtr->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setMaximum(m_ExtAxisPtr->getJointMaxAngle(jntID));
    m_ui->doubleSpinBox_lowerLimit->setValue(m_ExtAxisPtr->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setValue(m_ExtAxisPtr->getJointMaxAngle(jntID));
    m_ui->radioButton_rotDir_norm->setChecked(!m_ExtAxisPtr->isAxisDirInverted(jntID));
    m_ui->radioButton_rotDir_invrt->setChecked(m_ExtAxisPtr->isAxisDirInverted(jntID));
    m_ui->doubleSpinBox_lowerLimit->blockSignals(false);
    m_ui->doubleSpinBox_upperLimit->blockSignals(false);
}

void TaskBoxExtAxSetupPanel::slot_changeTargetJointLimits()
{
    auto jntID = m_ui->comboBox_JointList->currentIndex();
    auto lowerLimits = m_ExtAxisPtr->LowerLimits_Soft.getValues();
    lowerLimits[jntID] = m_ui->doubleSpinBox_lowerLimit->value();
    auto upperLimits = m_ExtAxisPtr->UpperLimits_Soft.getValues();
    upperLimits[jntID] = m_ui->doubleSpinBox_upperLimit->value();
    m_ExtAxisPtr->LowerLimits_Soft.setValues(lowerLimits);
    m_ExtAxisPtr->UpperLimits_Soft.setValues(upperLimits);
}

void TaskBoxExtAxSetupPanel::slot_importCalibData()
{
    QDialogCalibrationPanel* t_CalibDlg = new QDialogCalibrationPanel(m_ExtAxisPtr);
    QObject::connect(t_CalibDlg, SIGNAL(signal_updatePosePanel()),
                     this, SLOT(slot_updatePositionerPosePanel()));
    t_CalibDlg->show();
}

void TaskBoxExtAxSetupPanel::slot_flipTargetAxisDirection()
{
    auto axis_ID = m_ui->comboBox_JointList->currentIndex();
    m_ExtAxisPtr->flipAxisDirection(axis_ID, m_ui->radioButton_rotDir_invrt->isChecked());
}

void TaskBoxExtAxSetupPanel::slot_updateSliderPosition() {
  for (int jntID = 0; jntID < m_ExtAxisPtr->getJointNumbers(); jntID++) {
    auto sliderPtr = m_jointSliderVec[jntID];
    sliderPtr->updateAxisWidgetData(m_ExtAxisPtr->getJointAngle(jntID));
  }
}


void TaskBoxExtAxSetupPanel::slot_setPositionerToHomePose()
{
    m_ExtAxisPtr->resAxisHomePose();
}



void TaskBoxExtAxSetupPanel::slot_setCurrentPoseAsHomePosition() {
    m_ExtAxisPtr->setAxisHomePose();
}

void TaskBoxExtAxSetupPanel::slot_updatePositionerPose()
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
    m_ExtAxisPtr->Trans_Ref2Base.setValue(new_Trans);
}

void TaskBoxExtAxSetupPanel::slot_updatePositionerReference()
{
//    if(m_ui->comboBox_Reference->currentIndex()==0){
//        m_Positioner->Pose_Reference.setValue(Base::Placement());
////        m_Positioner->OriginReference.setValue("World");
//    }
//    else{
//        auto t_NameOfRefRBT = m_ui->comboBox_Reference->currentText().toStdString();
//        auto t_RefRbtPtr = static_cast<Robot::Robot6AxisObject*>(m_DocPtr->getObject(t_NameOfRefRBT.c_str()));
//        if(t_RefRbtPtr!=nullptr){
//            m_Positioner->Pose_Reference.setValue(t_RefRbtPtr->getCurrentBasePose());
////            m_Positioner->OriginReference.setValue(t_NameOfRefRBT.c_str());
//        }
//    }
//    m_ui->pushButton_BindReference->setEnabled(false);
}

void TaskBoxExtAxSetupPanel::slot_referenceTargetChanged()
{
    auto c_selected = m_ui->comboBox_Reference->currentText().toStdString();
//    m_ui->pushButton_BindReference->setEnabled(c_selected != m_Positioner->OriginReference.getStrValue());
}

void TaskBoxExtAxSetupPanel::slot_sliderPositionChanged(int t_Index) {
  if (m_jointSliderVec.empty())
    return;
  if (t_Index < m_jointSliderVec.size()) {
    auto sliderPtr = m_jointSliderVec[t_Index];
    m_ExtAxisPtr->setJointAngle(t_Index, sliderPtr->getSliderPosition());
  }
}

JointSliderWidget *TaskBoxExtAxSetupPanel::getTargetJointSlider(const size_t jntID) {
    if(jntID<0 || jntID>m_jointSliderVec.size())
        return nullptr;
    return m_jointSliderVec.at(jntID);
}


bool TaskBoxExtAxSetupPanel::accept() {
    m_ExtAxisPtr->Activated.setValue(false);
    return true;
}

bool TaskBoxExtAxSetupPanel::reject() {
    m_ExtAxisPtr->Activated.setValue(false);
    return true;
}



#include "moc_TaskBoxExtAxSetupPanel.cpp"
