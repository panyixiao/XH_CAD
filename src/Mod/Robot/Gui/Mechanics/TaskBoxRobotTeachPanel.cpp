#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBoxRobotTeachPanel.h"
#include <Base/Console.h>
#include <Gui/Document.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <QPushButton>
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/ui_TaskBoxRobotTeachPanel.h"
//#include "Mod/Robot/App/Mechanics/MechanicGroup.h"

using namespace RobotGui;

TaskBoxRobotTeachPanel::TaskBoxRobotTeachPanel(Robot::MechanicRobot *t_robot,
                                               Gui::TaskView::TaskSelectLinkProperty *t_FaceSelection,
                                               Gui::TaskView::TaskSelectLinkProperty *t_EdgeSelection)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Control Panel")) {
  if (t_robot == nullptr)
    return;
  m_DocPtr = t_robot->getDocument();
  m_RobotPtr = t_robot;
  m_FaceSelection = t_FaceSelection;
  m_EdgeSelection = t_EdgeSelection;
  initUi();
}

void TaskBoxRobotTeachPanel::initUi() {
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBoxRobotTeachPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);


  QObject::connect(m_ui->comboBox_TeachCoord, SIGNAL(currentIndexChanged(int)),
                   this,SLOT(slot_changeTeachCoord()));
  m_ui->comboBox_TeachCoord->setCurrentIndex(m_RobotPtr->getCurrentTeachCoord());
  QObject::connect(m_ui->pushButton_setHomePose, SIGNAL(clicked()),
                   this, SLOT(slot_setCurrentPoseAsHomePosition()));
  QObject::connect(m_ui->pushButton_toHomePose, SIGNAL(clicked()),
                   this, SLOT(slot_setRobotToHomePose()));

  // Up Panel
  initUi_AxisControllerBox();
  initUi_ConfigurationBox();
  initUi_PoserSetupBox();

  // Bot Panel
  initUi_TcpControlBox();
  initUi_BaseSetupBox();
}

void TaskBoxRobotTeachPanel::initUi_AxisControllerBox() {
  if(m_signalmapper == nullptr)
     m_signalmapper = new QSignalMapper(this);

  m_jointSliderVec.clear();
  uint slider_id = 0;
  // 初始化关节轴操作面板
  if(m_Layout_RobotJointPanel == nullptr)
     m_Layout_RobotJointPanel = new QGridLayout;
  for (size_t jntID = 0; jntID<m_RobotPtr->getJointNumbers(); jntID++)
  {
    auto jointName = std::string("J-")+std::to_string(jntID+1);
    auto slider_widget = new JointSliderWidget(m_ui->groupBox_RobotAxisControl,
                                               m_Layout_RobotJointPanel,
                                               jntID+1,slider_id,
                                               QString::fromStdString(jointName),
                                               this,
                                               m_signalmapper,
                                               m_RobotPtr->getJointMaxAngle(jntID),
                                               m_RobotPtr->getJointMinAngle(jntID));
    m_jointSliderVec.push_back(slider_widget);
    slider_id++;
  }
  if(m_RobotPtr->LinkedExtAxName.isEmpty())
    m_ui->groupBox_ExtAxisControl->setVisible(false);
  else{
      // 初始化外部轴操作面板
  }
  if(m_RobotPtr->LinkedPoserNames.getValues().empty())
    m_ui->groupBox_PoserAxisControl->setVisible(false);
  else{
      // 初始化变位机操作面板
      if(m_Layout_PoserJointPanel == nullptr)
         m_Layout_PoserJointPanel = new QGridLayout;
      uint poserNum = 1;
      for(const auto& linkName : m_RobotPtr->LinkedPoserNames.getValues()){
         auto poserPtr = m_RobotPtr->getTargetLinkedPoser(linkName);
         if(poserPtr!=nullptr){
             for (auto jntID = 0; jntID<poserPtr->getJointNumbers(); jntID++)
             {
               auto jointName = std::string("P")+std::to_string(poserNum) + std::string("-") + std::to_string(jntID+1);
               auto slider_widget = new JointSliderWidget(m_ui->groupBox_PoserAxisControl,
                                                          m_Layout_PoserJointPanel,
                                                          jntID+1,slider_id,
                                                          QString::fromStdString(jointName),
                                                          this,
                                                          m_signalmapper,
                                                          poserPtr->getJointMaxAngle(jntID),
                                                          poserPtr->getJointMinAngle(jntID));
               m_jointSliderVec.push_back(slider_widget);
               slider_id++;
             }
             poserNum++;
         }
      }
  }

  slot_updatePanelWidgets();
}

void TaskBoxRobotTeachPanel::initUi_ConfigurationBox()
{
    for(size_t i = 1; i<=m_RobotPtr->getJointNumbers(); i++){
        QString jnt_Name = QObject::tr("Jnt-")+QString::number(i);
        m_ui->comboBox_JointList->addItem(jnt_Name);
    }
    m_ui->doubleSpinBox_lowerLimit->setValue(m_RobotPtr->getJointMinAngle(0));
    m_ui->doubleSpinBox_lowerLimit->setMinimum(m_RobotPtr->getJointMinAngle(0));
    m_ui->doubleSpinBox_lowerLimit->setMaximum(m_RobotPtr->getJointMaxAngle(0));

    m_ui->doubleSpinBox_upperLimit->setValue(m_RobotPtr->getJointMaxAngle(0));
    m_ui->doubleSpinBox_upperLimit->setMinimum(m_RobotPtr->getJointMinAngle(0));
    m_ui->doubleSpinBox_upperLimit->setMaximum(m_RobotPtr->getJointMaxAngle(0));

    QObject::connect(m_ui->comboBox_JointList, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_targetConfigJointChanged()));
    QObject::connect(m_ui->pushButton_update, SIGNAL(clicked(bool)),
                     this, SLOT(slot_changeTargetJointLimits()));
    QObject::connect(m_ui->comboBox_config_forearm, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_changeArmConfig()));
    QObject::connect(m_ui->comboBox_config_elbow, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_changeArmConfig()));
    QObject::connect(m_ui->comboBox_config_wrist, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_changeArmConfig()));
    QObject::connect(m_ui->radioButton_AxisDir_Norm, SIGNAL(clicked(bool)),
                     this, SLOT(slot_flipAxisDirection()));
    QObject::connect(m_ui->radioButton_AxisDir_Invert, SIGNAL(clicked(bool)),
                     this, SLOT(slot_flipAxisDirection()));
    QObject::connect(m_ui->checkBox_enableConfigConstrain, SIGNAL(stateChanged(int)),
                     this, SLOT(slot_enableConfigConstraint()));
    slot_targetConfigJointChanged();
}

void TaskBoxRobotTeachPanel::initUi_PoserSetupBox()
{
    updateLinkedPoserWidgets();

    QObject::connect(m_ui->comboBox_poserList, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_targetPoserChanged()));
    QObject::connect(m_ui->pushButton_LinkPoser, SIGNAL(clicked(bool)),
                     this, SLOT(slot_poserButtonClicked()));
}

void TaskBoxRobotTeachPanel::initUi_TcpControlBox()
{
    if(m_RobotPtr->isDriven.getValue()){
//        m_ui->groupBox_ToolSetup->hide();
    }
    else{
//        m_ui->radioButton_Flan->setChecked(m_RobotPtr->CurrentToolIndex.getValue() == 0);
//        QObject::connect(m_ui->radioButton_Flan, SIGNAL(clicked()),this, SLOT(slot_changeActivatedTool()));

//        m_ui->radioButton_Torch->setEnabled(m_RobotPtr->hasTorch());
//        m_ui->radioButton_Torch->setChecked(m_RobotPtr->CurrentToolIndex.getValue() == m_RobotPtr->TorchIndex.getValue());
//        QObject::connect(m_ui->radioButton_Torch, SIGNAL(clicked()),this, SLOT(slot_changeActivatedTool()));

//        m_ui->radioButton_Scan->setEnabled(m_RobotPtr->hasScanner());
//        m_ui->radioButton_Scan->setChecked(m_RobotPtr->CurrentToolIndex.getValue() == m_RobotPtr->ScannerIndex.getValue());
//        m_ui->radioButton_Camera->setChecked(m_RobotPtr->CurrentToolIndex.getValue() == m_RobotPtr->CameraIndex.getValue());
//        QObject::connect(m_ui->radioButton_Scan, SIGNAL(clicked()),this, SLOT(slot_changeActivatedTool()));

//        QObject::connect(m_ui->radioButton_Camera, SIGNAL(clicked()),this, SLOT(slot_changeActivatedTool()));
//        QObject::connect(m_ui->spinBox_ToolID, SIGNAL(valueChanged(int)), SLOT(slot_changeToolIndex()));
    }


    QObject::connect(m_ui->doubleSpinBox_X, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetRobotTipPose()));
    QObject::connect(m_ui->doubleSpinBox_Y, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetRobotTipPose()));
    QObject::connect(m_ui->doubleSpinBox_Z, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetRobotTipPose()));

    QObject::connect(m_ui->doubleSpinBox_rZ, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetRobotTipPose()));
    QObject::connect(m_ui->doubleSpinBox_rY, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetRobotTipPose()));
    QObject::connect(m_ui->doubleSpinBox_rX, SIGNAL(valueChanged(double)), this, SLOT(slot_changeTargetRobotTipPose()));

}

void TaskBoxRobotTeachPanel::initUi_BaseSetupBox()
{
    auto extAxisDevice = m_DocPtr->getObjectsOfType(Robot::MechanicBase::getClassTypeId());

    for(auto objPtr : extAxisDevice){
        auto devicePtr = static_cast<Robot::MechanicBase*>(objPtr);
        if(devicePtr->DeviceType.getValue() == (int)Robot::MechanicType::M_ExtAxis)
            m_ui->comboBox_assembleReferenceList->addItem(tr(devicePtr->getNameInDocument()));
    }

    updateRef2BasePosePanel();
    QObject::connect(m_ui->pushButton_AssembleToTarget, SIGNAL(clicked()),
                     this,SLOT(slot_changeReferenceBase()));
    QObject::connect(m_ui->doubleSpinBox_Ref2Base_X, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeRef2BasePose()));
    QObject::connect(m_ui->doubleSpinBox_Ref2Base_Y, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeRef2BasePose()));
    QObject::connect(m_ui->doubleSpinBox_Ref2Base_Z, SIGNAL(valueChanged(double)),
                     this, SLOT(slot_changeRef2BasePose()));
    QObject::connect(m_ui->doubleSpinBox_Ref2Base_A, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_changeRef2BasePose()));
    QObject::connect(m_ui->doubleSpinBox_Ref2Base_B, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_changeRef2BasePose()));
    QObject::connect(m_ui->doubleSpinBox_Ref2Base_C, SIGNAL(valueChanged(double)), this,
                     SLOT(slot_changeRef2BasePose()));
}

void TaskBoxRobotTeachPanel::updateLinkedPoserWidgets()
{
    auto poserInDoc = m_DocPtr->getObjectsOfType(Robot::MechanicPoser::getClassTypeId());
    auto linkedPoserNames = m_RobotPtr->LinkedPoserNames.getValues();
    // 初始化变位机列表
    m_ui->tableWidget_LinkedPoser->clear();
    m_ui->tableWidget_LinkedPoser->setColumnCount(3);
    m_ui->tableWidget_LinkedPoser->setHorizontalHeaderLabels({tr("名称"),tr("轴数"),tr("负载")});
    size_t row_count = 0;
    for(const auto& t_Name : linkedPoserNames){
        auto deviceInfo = m_RobotPtr->getLinkedPoserInfo(t_Name);
        if(!deviceInfo.empty()){
            // 构建变位机参数
            QTableWidgetItem *name = new QTableWidgetItem(deviceInfo.at(0));
            QTableWidgetItem *axis = new QTableWidgetItem(deviceInfo.at(1));
            QTableWidgetItem *load = new QTableWidgetItem(deviceInfo.at(2));
            m_ui->tableWidget_LinkedPoser->insertRow(row_count);
            m_ui->tableWidget_LinkedPoser->setItem(row_count,0,name);
            m_ui->tableWidget_LinkedPoser->setItem(row_count,1,axis);
            m_ui->tableWidget_LinkedPoser->setItem(row_count,2,load);
            row_count++;
        }
    }
    m_ui->tableWidget_LinkedPoser->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    // 初始化可用的变位机列表
    m_ui->comboBox_poserList->clear();
    for(const auto& poserPtr : poserInDoc){
        m_ui->comboBox_poserList->addItem(tr(poserPtr->getNameInDocument()));
    }
    // 更新可用轴数
    QString content = QString::number(m_RobotPtr->getAvailablePoserAxisNumber()) + tr("/8");
    m_ui->label_availablePoserAxisNum->setText(content);
}

void TaskBoxRobotTeachPanel::updateRef2BasePosePanel()
{
    blockSignals_Basebox(true);
    m_ui->doubleSpinBox_Ref2Base_X->setValue(m_RobotPtr->Trans_Ref2Base.getValue().getPosition().x);
    m_ui->doubleSpinBox_Ref2Base_Y->setValue(m_RobotPtr->Trans_Ref2Base.getValue().getPosition().y);
    m_ui->doubleSpinBox_Ref2Base_Z->setValue(m_RobotPtr->Trans_Ref2Base.getValue().getPosition().z);
    double yaw, pit, roll;
    m_RobotPtr->Trans_Ref2Base.getValue().getRotation().getYawPitchRoll(yaw,pit,roll);
    m_ui->doubleSpinBox_Ref2Base_C->setValue(roll);
    m_ui->doubleSpinBox_Ref2Base_B->setValue(pit);
    m_ui->doubleSpinBox_Ref2Base_A->setValue(yaw);
    blockSignals_Basebox(false);
}

void TaskBoxRobotTeachPanel::slot_changeRef2BasePose() {
  double t_X = m_ui->doubleSpinBox_Ref2Base_X->value();
  double t_Y = m_ui->doubleSpinBox_Ref2Base_Y->value();
  double t_Z = m_ui->doubleSpinBox_Ref2Base_Z->value();
  auto trans_ = Base::Vector3d(t_X, t_Y, t_Z);
  auto new_Pose = Base::Placement();
  new_Pose.setPosition(trans_);
  auto rotate_ = Base::Rotation();
  rotate_.setYawPitchRoll(m_ui->doubleSpinBox_Ref2Base_A->value(),
                          m_ui->doubleSpinBox_Ref2Base_B->value(),
                          m_ui->doubleSpinBox_Ref2Base_C->value());
  new_Pose.setRotation(rotate_);
  m_RobotPtr->Trans_Ref2Base.setValue(new_Pose);
}

void TaskBoxRobotTeachPanel::slot_changeReferenceBase()
{
    try {
        if(m_ui->comboBox_assembleReferenceList->currentText() == tr("Selected Face")){
            if (m_FaceSelection->isSelectionValid()) {
              m_FaceSelection->sendSelection2Property();
            }
//            m_RobotPtr->setBaseToSelectedFaceCenter();    // This will be called once
        }
        else{
            if(!m_RobotPtr->LinkedExtAxName.getStrValue().empty()){
                auto t_DevicePtr = static_cast<Robot::MechanicBase*>(m_DocPtr->getObject(m_RobotPtr->LinkedExtAxName.getValue()));
//                t_DevicePtr->unloadRobot(m_RobotPtr->getNameInDocument());
            }

            auto extAxisDeviceName = m_ui->comboBox_assembleReferenceList->currentText().toStdString();
            auto t_DevicePtr = static_cast<Robot::MechanicBase*>(m_DocPtr->getObject(extAxisDeviceName.c_str()));
            if(t_DevicePtr!=nullptr && t_DevicePtr->DeviceType.getValue() == (int)Robot::MechanicType::M_ExtAxis){
//                t_DevicePtr->loadRobot(m_RobotPtr->getNameInDocument());
                m_RobotPtr->LinkedExtAxName.setValue(t_DevicePtr->getNameInDocument());
            }

        }

        updateRef2BasePosePanel();
    }
    catch (const Base::Exception &e) {
      Base::Console().Message(e.what());
    }
}

void TaskBoxRobotTeachPanel::slot_targetConfigJointChanged()
{
    auto jntID = m_ui->comboBox_JointList->currentIndex();
    m_ui->doubleSpinBox_lowerLimit->blockSignals(true);
    m_ui->doubleSpinBox_upperLimit->blockSignals(true);
    m_ui->doubleSpinBox_lowerLimit->setMinimum(m_RobotPtr->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_lowerLimit->setMaximum(m_RobotPtr->getJointMaxAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setMinimum(m_RobotPtr->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setMaximum(m_RobotPtr->getJointMaxAngle(jntID));
    m_ui->doubleSpinBox_lowerLimit->setValue(m_RobotPtr->getJointMinAngle(jntID));
    m_ui->doubleSpinBox_upperLimit->setValue(m_RobotPtr->getJointMaxAngle(jntID));
    m_ui->radioButton_AxisDir_Norm->setChecked(!m_RobotPtr->isAxisDirInverted(jntID));
    m_ui->radioButton_AxisDir_Invert->setChecked(m_RobotPtr->isAxisDirInverted(jntID));
    m_ui->doubleSpinBox_lowerLimit->blockSignals(false);
    m_ui->doubleSpinBox_upperLimit->blockSignals(false);
}

void TaskBoxRobotTeachPanel::slot_changeTargetJointLimits()
{
    auto jntID = m_ui->comboBox_JointList->currentIndex();
    auto lowerLimits = m_RobotPtr->LowerLimits_Soft.getValues();
    lowerLimits[jntID] = m_ui->doubleSpinBox_lowerLimit->value();
    auto upperLimits = m_RobotPtr->UpperLimits_Soft.getValues();
    upperLimits[jntID] = m_ui->doubleSpinBox_upperLimit->value();
    m_RobotPtr->LowerLimits_Soft.setValues(lowerLimits);
    m_RobotPtr->UpperLimits_Soft.setValues(upperLimits);
}

void TaskBoxRobotTeachPanel::slot_changeArmConfig()
{
//    m_RobotPtr->Wrist_NonFlip.setValue(m_ui->comboBox_config_wrist->currentText()==tr("NonFlip"));
//    m_RobotPtr->ForeArm_onRight.setValue(m_ui->comboBox_config_forearm->currentText()==tr("OnRight"));
//    m_RobotPtr->Elbow_Upward.setValue(m_ui->comboBox_config_elbow->currentText()==tr("Upward"));
}

void TaskBoxRobotTeachPanel::slot_flipAxisDirection()
{
    auto j_ID = m_ui->comboBox_JointList->currentIndex();
    m_RobotPtr->flipAxisDirection(j_ID, m_ui->radioButton_AxisDir_Invert->isChecked());
}

void TaskBoxRobotTeachPanel::slot_enableConfigConstraint()
{
    m_RobotPtr->EnableArmConfiguration.setValue(m_ui->checkBox_enableConfigConstrain->isChecked());
}

void TaskBoxRobotTeachPanel::slot_updatePanelWidgets() {
  for (size_t jntID = 0; jntID < m_RobotPtr->getJointNumbers(); jntID++) {
    auto sliderPtr = m_jointSliderVec[jntID];
    sliderPtr->updateAxisWidgetData(m_RobotPtr->getJointAngle(jntID));
  }
  slot_updateTipPosePanel();
}

void TaskBoxRobotTeachPanel::slot_updateTipPosePanel()
{
    Base::Placement t_Pose = m_RobotPtr->getCurrentTipPose(Robot::CoordOrigin::World);

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

void TaskBoxRobotTeachPanel::slot_changeTeachCoord()
{
    m_RobotPtr->setTeachCoordType(static_cast<Robot::TeachCoord>(m_ui->comboBox_TeachCoord->currentIndex()));
}

void TaskBoxRobotTeachPanel::slot_changeTargetRobotTipPose()
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

    m_RobotPtr->setRobotTipPose(t_Pose, Robot::CoordOrigin::World);
}

void TaskBoxRobotTeachPanel::slot_changeActivatedTool()
{
//    if(m_ui->radioButton_Flan->isChecked()){
//        m_RobotPtr->setCurrentToolType(Robot::ToolType::Undefined);
//        m_ui->spinBox_ToolID->setValue(0);
//    }
//    if(m_ui->radioButton_Torch->isChecked()){
//        m_RobotPtr->setCurrentToolType(Robot::ToolType::WeldTorch);
////        m_ui->spinBox_ToolID->setValue(m_RobotPtr->TorchIndex.getValue());
//    }
//    if(m_ui->radioButton_Scan->isChecked()){
//        m_RobotPtr->setCurrentToolType(Robot::ToolType::_2DScanner);
////        m_ui->spinBox_ToolID->setValue(m_RobotPtr->ScannerIndex.getValue());
//    }
//    if(m_ui->radioButton_Camera->isChecked()){
//        m_RobotPtr->setCurrentToolType(Robot::ToolType::_3DCamera);
////        m_ui->spinBox_ToolID->setValue(m_RobotPtr->CameraIndex.getValue());
//    }

    slot_updateTipPosePanel();
}

void TaskBoxRobotTeachPanel::slot_setRobotToHomePose()
{
    m_RobotPtr->resAxisHomePose();
}

void TaskBoxRobotTeachPanel::slot_targetPoserChanged()
{
    auto t_PoserName = m_ui->comboBox_poserList->currentText();
    auto linkedPoserNames = m_RobotPtr->LinkedPoserNames.getValues();
    auto result = std::find(linkedPoserNames.begin(),linkedPoserNames.end(),t_PoserName.toStdString());
    if(result == linkedPoserNames.end()){
        m_ui->pushButton_LinkPoser->setText(tr("绑定"));
        operationFlag_bind = true;
    }
    else{
        m_ui->pushButton_LinkPoser->setText(tr("解绑"));
        operationFlag_bind = false;
    }
}

void TaskBoxRobotTeachPanel::slot_poserButtonClicked()
{
    auto t_PoserName = m_ui->comboBox_poserList->currentText();
    auto linkedPoserNames = m_RobotPtr->LinkedPoserNames.getValues();
    auto result = std::find(linkedPoserNames.begin(),linkedPoserNames.end(),t_PoserName.toStdString());
    if(result == linkedPoserNames.end()){
        linkedPoserNames.push_back(t_PoserName.toStdString());
    }else{
        linkedPoserNames.erase(result);
    }
    m_RobotPtr->LinkedPoserNames.setValues(linkedPoserNames);
    updateLinkedPoserWidgets();
}

void TaskBoxRobotTeachPanel::slot_changeToolIndex()
{
//    int val = m_ui->spinBox_ToolID->value();
//    switch(m_RobotPtr->getCurrentTool()){
//    case Robot::ToolType::WeldTorch:
////        m_RobotPtr->TorchIndex.setValue(val);
//        break;
//    case Robot::ToolType::_2DScanner:
////        m_RobotPtr->ScannerIndex.setValue(val);
//        break;
//    case Robot::ToolType::_3DCamera:
////        m_RobotPtr->CameraIndex.setValue(val);
//        break;
//    }
}

void TaskBoxRobotTeachPanel::slot_openToolSetupBox()
{
    Q_EMIT Signal_setupRobotTool();
}


void TaskBoxRobotTeachPanel::slot_setCurrentPoseAsHomePosition() {
  m_RobotPtr->setAxisHomePose();
}

void TaskBoxRobotTeachPanel::slot_sliderPositionChanged(int t_Index) {
  if (m_jointSliderVec.empty())
    return;
  if (t_Index < m_jointSliderVec.size()) {
    auto sliderPtr = m_jointSliderVec[t_Index];
    m_RobotPtr->setJointAngle(t_Index, sliderPtr->getSliderPosition());
  }
}

JointSliderWidget *TaskBoxRobotTeachPanel::findTargetJointSlider(const size_t jntID) {
    if(jntID<0 || jntID>m_jointSliderVec.size())
        return nullptr;
    return m_jointSliderVec.at(jntID);
}

void TaskBoxRobotTeachPanel::blockSignals_TCPbox(bool b)
{
    m_ui->doubleSpinBox_X->blockSignals(b);
    m_ui->doubleSpinBox_Y->blockSignals(b);
    m_ui->doubleSpinBox_Z->blockSignals(b);

    m_ui->doubleSpinBox_rX->blockSignals(b);
    m_ui->doubleSpinBox_rY->blockSignals(b);
    m_ui->doubleSpinBox_rZ->blockSignals(b);
}

void TaskBoxRobotTeachPanel::blockSignals_Basebox(bool b)
{
    m_ui->doubleSpinBox_Ref2Base_X->blockSignals(b);
    m_ui->doubleSpinBox_Ref2Base_Y->blockSignals(b);
    m_ui->doubleSpinBox_Ref2Base_Z->blockSignals(b);
    m_ui->doubleSpinBox_Ref2Base_C->blockSignals(b);
    m_ui->doubleSpinBox_Ref2Base_B->blockSignals(b);
    m_ui->doubleSpinBox_Ref2Base_A->blockSignals(b);
}


bool TaskBoxRobotTeachPanel::accept() {
    m_RobotPtr->Activated.setValue(false);
    return true;
}

bool TaskBoxRobotTeachPanel::reject() {
    m_RobotPtr->Activated.setValue(false);
    return true;
}

#include "moc_TaskBoxRobotTeachPanel.cpp"
