// Created By Yixiao 2022-08-15

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBoxMechanicGroupPanel.h"
#include <Base/Console.h>
#include <Gui/Document.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <QPushButton>
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxMechanicGroupPanel.h"
#include "Mod/Robot/App/Mechanics/MechanicGroup.h"
#include "Mod/Robot/Gui/Mechanics/ViewProviderMechanicGroup.h"

using namespace RobotGui;

TaskBoxMechanicGroupPanel::TaskBoxMechanicGroupPanel(Robot::MechanicGroup *t_Group)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Control Panel")) {
  if (t_Group == nullptr)
    return;
  m_DocPtr = t_Group->getDocument();
  m_MechGroup = t_Group;
  initUi_Panel();
}

void TaskBoxMechanicGroupPanel::initUi_Panel() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxMechanicGroupPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  QObject::connect(m_ui->comboBox_TeachCoord, SIGNAL(currentIndexChanged(int)),
                   this,SLOT(slot_changeTeachCoord()));
  m_ui->comboBox_TeachCoord->setCurrentIndex(m_MechGroup->getCurrentTeachCoord());

  QObject::connect(m_ui->pushButton_setHomePose, SIGNAL(clicked()),
                   this, SLOT(slot_setCurrentPoseAsHomePosition()));
  QObject::connect(m_ui->pushButton_toHomePose, SIGNAL(clicked()),
                   this, SLOT(slot_setRobotToHomePose()));

  initUi_AxisControllerBox();
  initUi_TcpControlBox();
  initUi_BodySetupBox();
  initUi_CommSetupBox();
}

void TaskBoxMechanicGroupPanel::initUi_AxisControllerBox() {

  if(m_signalmapper == nullptr)
     m_signalmapper = new QSignalMapper(this);

  m_jointSliderVec.clear();

  QGridLayout * tab1_layout = new QGridLayout();
  for (int jntID = 0; jntID<8; jntID++)
  {
    auto jntName = std::string("R1J")+std::to_string(jntID+1);
    auto u_limit = m_MechGroup->getJointMaxAngle(jntID);
    auto l_limit = m_MechGroup->getJointMinAngle(jntID);
    auto slider_widget = new JointSliderWidget(m_ui->tab_Robot1AxisControl,
                                               tab1_layout,
                                               jntID+1,jntID,
                                               QString::fromStdString(jntName),
                                               this,
                                               m_signalmapper,
                                               u_limit, l_limit);
    m_jointSliderVec.push_back(slider_widget);
  }

  QGridLayout * tab2_layout = new QGridLayout();
  for (int jntID = 8; jntID<16; jntID++)
  {
    auto jntName = std::string("R2J")+std::to_string(jntID-7);
    auto u_limit = m_MechGroup->getJointMaxAngle(jntID);
    auto l_limit = m_MechGroup->getJointMinAngle(jntID);
    auto slider_widget = new JointSliderWidget(m_ui->tab_Robot2AxisControl,
                                               tab2_layout,
                                               jntID-7,jntID,
                                               QString::fromStdString(jntName),
                                               this,
                                               m_signalmapper,
                                               u_limit, l_limit);
    m_jointSliderVec.push_back(slider_widget);
  }

  QGridLayout * tab3_layout = new QGridLayout();
  for (int jntID = 16; jntID<24; jntID++)
  {
    auto jntName = std::string("Ext-")+std::to_string(jntID-15);
    auto u_limit = m_MechGroup->getJointMaxAngle(jntID);
    auto l_limit = m_MechGroup->getJointMinAngle(jntID);
    auto slider_widget = new JointSliderWidget(m_ui->tab_ExtAxisControl,
                                               tab3_layout,
                                               jntID-15,jntID,
                                               QString::fromStdString(jntName),
                                               this,
                                               m_signalmapper,
                                               u_limit, l_limit);
    m_jointSliderVec.push_back(slider_widget);
  }

  slot_updatePanelWidgets();

  auto c_ID = m_MechGroup->ActiveRobotIndex.getValue();
  m_ui->tabWidget_ControlTarget->setCurrentIndex(c_ID-1);
  QObject::connect(m_ui->pushButton_setAcitveRobot, SIGNAL(clicked(bool)),this, SLOT(slot_setAcitveRobot()));
  QObject::connect(m_ui->tabWidget_ControlTarget, SIGNAL(currentChanged(int)),this, SLOT(slot_controlTargetChanged(int)));
}


void TaskBoxMechanicGroupPanel::initUi_TcpControlBox()
{
    QObject::connect(m_ui->checkBox_setDraggerPrior, SIGNAL(clicked(bool)),this, SLOT(slot_setDraggerAboveAll()));

    QObject::connect(m_ui->doubleSpinBox_X, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetTipPose()));
    QObject::connect(m_ui->doubleSpinBox_Y, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetTipPose()));
    QObject::connect(m_ui->doubleSpinBox_Z, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetTipPose()));

    QObject::connect(m_ui->doubleSpinBox_rZ, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetTipPose()));
    QObject::connect(m_ui->doubleSpinBox_rY, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetTipPose()));
    QObject::connect(m_ui->doubleSpinBox_rX, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetTipPose()));

}

void TaskBoxMechanicGroupPanel::initUi_BodySetupBox()
{
    // Tab Robot
    updateRobotList();
    updateLinkedRobotLabel();
    QObject::connect(m_ui->pushButton_bindRobot, SIGNAL(clicked(bool)),this, SLOT(slot_linkTargetRobot()));
    QObject::connect(m_ui->pushButton_RemoveRobot1, SIGNAL(clicked(bool)),this, SLOT(slot_resetLinkedRobot1()));
    QObject::connect(m_ui->pushButton_RemoveRobot2, SIGNAL(clicked(bool)),this, SLOT(slot_resetLinkedRobot2()));

    // Tab ExtAxis
    updatePoserList();
    m_ui->tableWidget_ExtAxisList->horizontalHeader()->setStretchLastSection(true);
    QObject::connect(m_ui->pushButton_BindExtAxis, SIGNAL(clicked(bool)),this, SLOT(slot_bindTargetPoser()));
    QObject::connect(m_ui->pushButton_resetExtAxis, SIGNAL(clicked(bool)),this, SLOT(slot_resetLinkedPosers()));
    updateLinkedPoserAxisTable();
}

void TaskBoxMechanicGroupPanel::initUi_CommSetupBox()
{
    QObject::connect(m_ui->pushButton_connectToStation, SIGNAL(clicked(bool)),
                     this, SLOT(slot_pushButtonConnectClicked()));
    QObject::connect(m_ui->lineEdit_stationIP, SIGNAL(textChanged(QString)),
                     this, SLOT(slot_networkAddressChanged()));
    QObject::connect(m_ui->lineEdit_stationPortNum,SIGNAL(textChanged(QString)),
                     this, SLOT(slot_networkAddressChanged()));
    QObject::connect(m_ui->checkBox_updatePoseFromStation,SIGNAL(stateChanged(int)),
                     this, SLOT(slot_enablePoseUpdating()));
    m_ui->checkBox_updatePoseFromStation->setEnabled(false);
    slot_networkAddressChanged();
}

void TaskBoxMechanicGroupPanel::updateRobotList()
{
    m_ui->comboBox_robotList->clear();
    auto robotPtrList = m_DocPtr->getObjectsOfType(Robot::Robot6AxisObject::getClassTypeId());
    for(auto robotPtr : robotPtrList){
        if(robotPtr!=m_MechGroup->getLinkedRobot1Ptr() && robotPtr!=m_MechGroup->getLinkedRobot2Ptr()){
            m_ui->comboBox_robotList->addItem(tr(robotPtr->getNameInDocument()));
        }
    }
}

void TaskBoxMechanicGroupPanel::updateLinkedRobotLabel()
{
    m_ui->label_LinkedRbtName_1->setText(tr(m_MechGroup->LinkedRobotName_1.getValue()));
    m_ui->label_LinkedRbtName_2->setText(tr(m_MechGroup->LinkedRobotName_2.getValue()));
}

void TaskBoxMechanicGroupPanel::updatePoserList()
{
    m_ui->comboBox_extAxisList->clear();
    auto poserPtrList = m_DocPtr->getObjectsOfType(Robot::MechanicDevice::getClassTypeId());
    auto linkedPoserList = m_MechGroup->LinkedPoserNames.getValues();
     for(auto poserPtr : poserPtrList){
         bool linked = false;
         auto t_poserName = std::string(poserPtr->getNameInDocument());
         for(auto poserName : linkedPoserList){
             if(t_poserName == poserName){
                 linked = true;
             }
         }
         if(!linked){
             m_ui->comboBox_extAxisList->addItem(tr(t_poserName.c_str()));
         }
     }
}

void TaskBoxMechanicGroupPanel::updateLinkedPoserAxisTable(){
    m_ui->tableWidget_ExtAxisList->clearContents();
    uint row_id = 0;
    uint row_count = 0;
    for(auto t_Poser : m_MechGroup->getLinkedPositioner()){
        auto axisNames = t_Poser->getAxisNames();
        row_count+=axisNames.size();
        m_ui->tableWidget_ExtAxisList->setRowCount(row_count);
        for(auto name : axisNames){
            QTableWidgetItem* t_Name = new QTableWidgetItem;
            t_Name->setText(tr(name.c_str()));
            m_ui->tableWidget_ExtAxisList->setItem(row_id, 0, t_Name);
            row_id++;
        }
    }
}


void TaskBoxMechanicGroupPanel::slot_updatePanelWidgets() {
  for (size_t jntID = 0; jntID < m_MechGroup->getJointNumbers(); jntID++) {
    auto sliderPtr = m_jointSliderVec[jntID];
    sliderPtr->updateAxisWidgetData(m_MechGroup->getJointAngle(jntID));
  }
  slot_updateTipPosePanel();
}

void TaskBoxMechanicGroupPanel::slot_stationConnected(bool flag)
{
    m_ui->pushButton_connectToStation->setEnabled(true);
    if(flag){
        m_ui->pushButton_connectToStation->setText(tr("Disconnect from Station"));
        m_ui->checkBox_updatePoseFromStation->setEnabled(true);
        m_ui->checkBox_updatePoseFromStation->setChecked(true);
    }
    else{
        m_ui->pushButton_connectToStation->setText(tr("Connect to Station"));
    }
}

void TaskBoxMechanicGroupPanel::slot_updateTipPosePanel()
{
    Base::Placement t_Pose = m_MechGroup->getGroupTipPose();

    blockSignals_TCPbox(true);
    m_ui->doubleSpinBox_X->setValue(t_Pose.getPosition().x);
    m_ui->doubleSpinBox_Y->setValue(t_Pose.getPosition().y);
    m_ui->doubleSpinBox_Z->setValue(t_Pose.getPosition().z);
    double y,p,r;
    t_Pose.getRotation().getYawPitchRoll(y,p,r);
    m_ui->doubleSpinBox_rZ->setValue(y);
    m_ui->doubleSpinBox_rY->setValue(p);
    m_ui->doubleSpinBox_rX->setValue(r);

    blockSignals_TCPbox(false);
}

void TaskBoxMechanicGroupPanel::slot_changeTeachCoord()
{
    m_MechGroup->setTeachCoordType(static_cast<Robot::TeachCoord>(m_ui->comboBox_TeachCoord->currentIndex()));
    slot_updateTipPosePanel();
}

void TaskBoxMechanicGroupPanel::slot_changeTargetTipPose()
{
    Base::Placement t_Pose;
    t_Pose.setPosition(Base::Vector3d(m_ui->doubleSpinBox_X->value(),
                                      m_ui->doubleSpinBox_Y->value(),
                                      m_ui->doubleSpinBox_Z->value()));
    Base::Rotation t_Rot;
    t_Rot.setYawPitchRoll(m_ui->doubleSpinBox_rZ->value(),
                          m_ui->doubleSpinBox_rY->value(),
                          m_ui->doubleSpinBox_rX->value());
    t_Pose.setRotation(t_Rot);

    m_MechGroup->setTipPose(t_Pose, Robot::CoordOrigin::World);
}

void TaskBoxMechanicGroupPanel::slot_changeActivatedTool()
{
    slot_updateTipPosePanel();
}

void TaskBoxMechanicGroupPanel::slot_setRobotToHomePose()
{
    m_MechGroup->restoreHomePose();
}

void TaskBoxMechanicGroupPanel::slot_setAcitveRobot()
{
    uint t_ID = m_ui->tabWidget_ControlTarget->currentIndex();
    if(t_ID < 2){
        m_MechGroup->setActiveRobot(t_ID+1);
    }
}

void TaskBoxMechanicGroupPanel::slot_controlTargetChanged(int index)
{
    if(index == 0)
        m_ui->pushButton_setAcitveRobot->setEnabled(!m_MechGroup->LinkedRobotName_1.isEmpty());
    else if(index == 1)
        m_ui->pushButton_setAcitveRobot->setEnabled(!m_MechGroup->LinkedRobotName_2.isEmpty());
    else
        m_ui->pushButton_setAcitveRobot->setEnabled(false);
}

void TaskBoxMechanicGroupPanel::slot_changeToolIndex()
{
//    m_targetGroup->setCurrentToolType();
}


void TaskBoxMechanicGroupPanel::slot_setCurrentPoseAsHomePosition() {
    m_MechGroup->setCurrentPoseAsHome();
}

void TaskBoxMechanicGroupPanel::slot_linkTargetRobot()
{
    auto t_Name = m_ui->comboBox_robotList->currentText().toStdString();
    auto t_RobotPtr = m_DocPtr->getObject(t_Name.c_str());
    if(t_RobotPtr == nullptr)
        return;
    if(m_MechGroup->getLinkedRobot1Ptr() == nullptr){
        m_MechGroup->LinkedRobotName_1.setValue(t_Name);
    }
    else if(m_MechGroup->getLinkedRobot2Ptr() == nullptr){
        m_MechGroup->LinkedRobotName_2.setValue(t_Name);
    }
    updateLinkedRobotLabel();
    updateRobotList();
}

void TaskBoxMechanicGroupPanel::slot_bindTargetPoser()
{
    auto linkedPoserNames = m_MechGroup->LinkedPoserNames.getValues();
    linkedPoserNames.push_back(m_ui->comboBox_extAxisList->currentText().toStdString());
    m_MechGroup->LinkedPoserNames.setValues(linkedPoserNames);
    updatePoserList();
    updateLinkedPoserAxisTable();
}

void TaskBoxMechanicGroupPanel::slot_resetLinkedRobot1()
{
    m_MechGroup->LinkedRobotName_1.setValue("");
    m_ui->label_LinkedRbtName_1->setText(tr(""));
    updateRobotList();
}

void TaskBoxMechanicGroupPanel::slot_resetLinkedRobot2()
{
    m_MechGroup->LinkedRobotName_2.setValue("");
    m_ui->label_LinkedRbtName_2->setText(tr(""));
    updateRobotList();
}

void TaskBoxMechanicGroupPanel::slot_resetLinkedPosers()
{
    m_MechGroup->LinkedPoserNames.setValues(std::list<std::string>());
    updatePoserList();
}

void TaskBoxMechanicGroupPanel::slot_setDraggerAboveAll()
{
    if(m_MechGroup == nullptr)
        return;
    auto m_GroupVP = static_cast<ViewProviderMechanicGroup*>(Gui::Application::Instance->activeDocument()->getViewProvider(m_MechGroup));
    if(m_GroupVP == nullptr)
        return;
    m_GroupVP->EnableSelection.setValue(!m_ui->checkBox_setDraggerPrior->isChecked());
}

void TaskBoxMechanicGroupPanel::slot_pushButtonConnectClicked()
{
    if(m_MechGroup->stationConnected()){
        m_ui->checkBox_updatePoseFromStation->setChecked(false);
        m_ui->checkBox_updatePoseFromStation->setEnabled(false);
        m_MechGroup->disconnectRobot();
    }
    else{
        m_MechGroup->connectToRobot();
    }
}

void TaskBoxMechanicGroupPanel::slot_networkAddressChanged()
{
    auto t_IP = m_ui->lineEdit_stationIP->text().toStdString();
    auto t_Port = m_ui->lineEdit_stationPortNum->text().toInt();
    m_MechGroup->setStationAddress(t_IP,t_Port);
}

void TaskBoxMechanicGroupPanel::slot_enablePoseUpdating()
{
    if(m_ui->checkBox_updatePoseFromStation->isChecked())
        m_MechGroup->startUpdatePose();
    else
        m_MechGroup->stopUpdatePose();
}

void TaskBoxMechanicGroupPanel::sliderPositionChanged(int t_Index) {
  if (m_jointSliderVec.empty())
    return;
  if (t_Index < m_jointSliderVec.size()) {
    auto sliderPtr = m_jointSliderVec[t_Index];
    m_MechGroup->setJointAngle(t_Index, sliderPtr->getSliderPosition());
//    sliderPtr->set_labelvalue();
  }
}

JointSliderWidget *TaskBoxMechanicGroupPanel::findTargetJointSlider(const int jntID) {
    if(jntID<0 || jntID>m_jointSliderVec.size())
        return nullptr;
    return m_jointSliderVec.at(jntID);
}

void TaskBoxMechanicGroupPanel::blockSignals_TCPbox(bool b)
{
    m_ui->doubleSpinBox_X->blockSignals(b);
    m_ui->doubleSpinBox_Y->blockSignals(b);
    m_ui->doubleSpinBox_Z->blockSignals(b);

    m_ui->doubleSpinBox_rX->blockSignals(b);
    m_ui->doubleSpinBox_rY->blockSignals(b);
    m_ui->doubleSpinBox_rZ->blockSignals(b);
}

bool TaskBoxMechanicGroupPanel::accept() {
    m_MechGroup->Activated.setValue(false);
    return true;
}

bool TaskBoxMechanicGroupPanel::reject() {
    m_MechGroup->Activated.setValue(false);
    return true;
}

#include "moc_TaskBoxMechanicGroupPanel.cpp"
