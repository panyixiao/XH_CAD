// Created by Yixiao 2022/04/25

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Control.h>
#include <Gui/Command.h>

#include "TaskDlgPlanningObject.h"
#include "Mod/Robot/Gui/ui_TaskDlgPlanningObject.h"
#include "Mod/Robot/App/Mechanics/MechanicPoser.h"
#include <Mod/Robot/Gui/PlanningObj/ViewProviderPlanningObj.h>
#include <QMessageBox>

using namespace RobotGui;

TaskDlgPlanningObject::TaskDlgPlanningObject(Robot::PlanningObject *t_PlanningObj,
                                             QWidget *parent)
    : m_ui(new Ui_TaskDlgPlanningObject), TaskDialog() {
  if (t_PlanningObj == nullptr)
    return;
  m_Doc = t_PlanningObj->getDocument();
  m_PlanningObj = t_PlanningObj;
  m_PlanningObj_VP = dynamic_cast<ViewProviderPlanningObj*>(Gui::Application::Instance->activeDocument()->getViewProvider(m_PlanningObj));
  initUi();
  createDragger(m_PlanningObj->getCurrentMountPose());
}

bool TaskDlgPlanningObject::accept() {
    Gui::Document *document = Gui::Application::Instance->getDocument(m_Doc);
    if (!document)
      return false;
    document->commitCommand();
    document->resetEdit();
    return true;
}

bool TaskDlgPlanningObject::reject() {
    Gui::Document *document = Gui::Application::Instance->getDocument(m_Doc);
    if (!document)
      return false;
    document->commitCommand();
    document->resetEdit();
    return true;
}

void TaskDlgPlanningObject::initUi() {
  Gui::TaskView::TaskBox *t_TaskBox = new Gui::TaskView::TaskBox(Gui::BitmapFactory().pixmap("document-new"),
                                                                 tr("Object Property"), true, 0);

  m_EdgeSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Edge COUNT 1",
                                                              &(m_PlanningObj->tCurvFeature));
  t_TaskBox->groupLayout()->addWidget(m_EdgeSelection);
  m_EdgeSelection->hide();
  Content.push_back(m_EdgeSelection);

  m_FaceSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Face COUNT 1",
                                                              &(m_PlanningObj->tFaceFeature));
  t_TaskBox->groupLayout()->addWidget(m_FaceSelection);
  m_FaceSelection->hide();
  Content.push_back(m_FaceSelection);

  m_proxy = new QWidget();
  m_ui->setupUi(m_proxy);
  t_TaskBox->groupLayout()->addWidget(m_proxy);
  Content.push_back(t_TaskBox);

  updateMountTargetDeviceList();
  QObject::connect(m_ui->tabWidget_EditPanel, SIGNAL(currentChanged(int)), this,
                   SLOT(slot_editTargetChanged(int)));
  QObject::connect(m_ui->doubleSpinBox_Pose_tX, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeMountPose()));
  QObject::connect(m_ui->doubleSpinBox_Pose_tY, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeMountPose()));
  QObject::connect(m_ui->doubleSpinBox_Pose_tZ, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeMountPose()));
  QObject::connect(m_ui->doubleSpinBox_Pose_rX, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeMountPose()));
  QObject::connect(m_ui->doubleSpinBox_Pose_rY, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeMountPose()));
  QObject::connect(m_ui->doubleSpinBox_Pose_rZ, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeMountPose()));
  QObject::connect(m_ui->pushButton_reCapCenter, SIGNAL(clicked(bool)),
                   this, SLOT(slot_setMountPoseToFeatureCenter()));
  QObject::connect(m_ui->pushButton_Assemble,SIGNAL(clicked()), this,
                   SLOT(slot_changeMountState()));
  QObject::connect(m_ui->pushButton_FlipPoseRx, SIGNAL(clicked(bool)),
                   this, SLOT(slot_flipAngle_Mount_rX()));
  QObject::connect(m_ui->pushButton_FlipPoseRy, SIGNAL(clicked(bool)),
                   this, SLOT(slot_flipAngle_Mount_rY()));
  QObject::connect(m_ui->pushButton_FlipPoseRz, SIGNAL(clicked(bool)),
                   this, SLOT(slot_flipAngle_Mount_rZ()));
  updateMountPanelInformation();

  // FramePose Setup Panel
  QObject::connect(m_ui->pushButton_FrameSwitch, SIGNAL(clicked(bool)),
                   this, SLOT(slot_changeFrameStatus()));
  updateFramePanelStatus();
  QObject::connect(m_ui->pushButton_setFrameToCenter, SIGNAL(clicked(bool)),
                   this, SLOT(slot_setFramePoseToFeatureCenter()));
  QObject::connect(m_ui->pushButton_DoneEdit, SIGNAL(clicked(bool)),
                   this, SLOT(slot_finishEditObject()));
  QObject::connect(m_ui->doubleSpinBox_Frame_tX, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_changeFramePose()));
  QObject::connect(m_ui->doubleSpinBox_Frame_tY, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_changeFramePose()));
  QObject::connect(m_ui->doubleSpinBox_Frame_tZ, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_changeFramePose()));
  QObject::connect(m_ui->doubleSpinBox_Frame_rX, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_changeFramePose()));
  QObject::connect(m_ui->doubleSpinBox_Frame_rY, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_changeFramePose()));
  QObject::connect(m_ui->doubleSpinBox_Frame_rZ, SIGNAL(valueChanged(double)),
                   this, SLOT(slot_changeFramePose()));
  QObject::connect(m_ui->pushButton_FlipFrameRx, SIGNAL(clicked(bool)),
                   this, SLOT(slot_flipAngle_Frame_rX()));
  QObject::connect(m_ui->pushButton_FlipFrameRy, SIGNAL(clicked(bool)),
                   this, SLOT(slot_flipAngle_Frame_rY()));
  QObject::connect(m_ui->pushButton_FlipFrameRz, SIGNAL(clicked(bool)),
                   this, SLOT(slot_flipAngle_Frame_rZ()));
  updateFramePoseInformation();
}

void TaskDlgPlanningObject::blockMountPosePanelSignal(bool blocking) {
  m_ui->doubleSpinBox_Pose_tX->blockSignals(blocking);
  m_ui->doubleSpinBox_Pose_tY->blockSignals(blocking);
  m_ui->doubleSpinBox_Pose_tZ->blockSignals(blocking);
  m_ui->doubleSpinBox_Pose_rX->blockSignals(blocking);
  m_ui->doubleSpinBox_Pose_rY->blockSignals(blocking);
  m_ui->doubleSpinBox_Pose_rZ->blockSignals(blocking);
}

void TaskDlgPlanningObject::blockFramePosePanelSignal(bool blocking)
{
    m_ui->doubleSpinBox_Frame_tX->blockSignals(blocking);
    m_ui->doubleSpinBox_Frame_tY->blockSignals(blocking);
    m_ui->doubleSpinBox_Frame_tZ->blockSignals(blocking);
    m_ui->doubleSpinBox_Frame_rX->blockSignals(blocking);
    m_ui->doubleSpinBox_Frame_rY->blockSignals(blocking);
    m_ui->doubleSpinBox_Frame_rZ->blockSignals(blocking);
}

void TaskDlgPlanningObject::updateMountPanelInformation() {
  if (m_PlanningObj == nullptr)
    return;
  blockMountPosePanelSignal(true);
  m_ui->doubleSpinBox_Pose_tX->setValue(m_PlanningObj->Trans_O2M.getValue().getPosition().x);
  m_ui->doubleSpinBox_Pose_tY->setValue(m_PlanningObj->Trans_O2M.getValue().getPosition().y);
  m_ui->doubleSpinBox_Pose_tZ->setValue(m_PlanningObj->Trans_O2M.getValue().getPosition().z);
  double r_y, r_p, r_r;
  m_PlanningObj->Trans_O2M.getValue().getRotation().getYawPitchRoll(r_y, r_p, r_r);
  m_ui->doubleSpinBox_Pose_rX->setValue(r_p);
  m_ui->doubleSpinBox_Pose_rY->setValue(r_r);
  m_ui->doubleSpinBox_Pose_rZ->setValue(r_y);
  blockMountPosePanelSignal(false);
  updateDraggerPose(m_PlanningObj->getCurrentMountPose());
}

void TaskDlgPlanningObject::updateMountTargetDeviceList() {
    m_ui->comboBox_TargetDeivice->blockSignals(true);
    if(m_PlanningObj->AttachedTo.getStrValue().empty()){
        m_ui->comboBox_TargetDeivice->clear();
        m_ui->pushButton_Assemble->setText(tr("Mount"));
        auto t_ObjectList = m_Doc->getObjects();
        uint count = 0;
        for(auto t_obj : t_ObjectList){
            bool validObj = false;
            if(t_obj->isDerivedFrom(Robot::ToolObject::getClassTypeId())){
                auto t_ToolPtr = static_cast<Robot::ToolObject*>(t_obj);
                if(t_ToolPtr->_ToolType.getValue() == (uint)Robot::ToolType::Gripper)
                    validObj = true;
            }
            else if(t_obj->isDerivedFrom(Robot::MechanicPoser::getClassTypeId())){
                validObj = true;
            }
            if(validObj){
                m_ui->comboBox_TargetDeivice->addItem(tr(t_obj->getNameInDocument()));
                count++;
            }
        }
        m_ui->pushButton_Assemble->setEnabled(count!=0);
    }
    else{
        m_ui->comboBox_TargetDeivice->addItem(tr(m_PlanningObj->AttachedTo.getValue()));
        m_ui->pushButton_Assemble->setText(tr("Dismount"));
    }
    m_ui->comboBox_TargetDeivice->blockSignals(false);
}

void TaskDlgPlanningObject::updateFramePoseInformation()
{
    blockFramePosePanelSignal(true);
    auto t_FrameTrans = m_PlanningObj->Trans_O2F.getValue();
    m_ui->doubleSpinBox_Frame_tX->setValue(t_FrameTrans.getPosition().x);
    m_ui->doubleSpinBox_Frame_tY->setValue(t_FrameTrans.getPosition().y);
    m_ui->doubleSpinBox_Frame_tZ->setValue(t_FrameTrans.getPosition().z);
    double rX,rY,rZ;
    t_FrameTrans.getRotation().getYawPitchRoll(rZ,rY,rX);
    m_ui->doubleSpinBox_Frame_rZ->setValue(rZ);
    m_ui->doubleSpinBox_Frame_rY->setValue(rY);
    m_ui->doubleSpinBox_Frame_rX->setValue(rX);
    blockFramePosePanelSignal(false);
    updateDraggerPose(m_PlanningObj->getCurrentFramePose());
}

void TaskDlgPlanningObject::slot_setMountPoseToFeatureCenter() {
  if (m_EdgeSelection->isSelectionValid()) {
    m_EdgeSelection->sendSelection2Property();
  }
  if (m_FaceSelection->isSelectionValid()) {
    m_FaceSelection->sendSelection2Property();
  }
  if (m_PlanningObj != nullptr) {
    m_PlanningObj->setMountPose_toFeatureCenter();
  }
  updateMountPanelInformation();
}

void TaskDlgPlanningObject::slot_setFramePoseToFeatureCenter()
{
    if (m_EdgeSelection->isSelectionValid()) {
      m_EdgeSelection->sendSelection2Property();
    }
    if (m_FaceSelection->isSelectionValid()) {
      m_FaceSelection->sendSelection2Property();
    }
    if (m_PlanningObj != nullptr) {
        m_PlanningObj->setFramePose_toFeatureCenter();
    }
    updateFramePoseInformation();
}

void TaskDlgPlanningObject::slot_changeMountState()
{
    auto t_Name = m_ui->comboBox_TargetDeivice->currentText().toStdString();
    if(t_Name.empty())
        return;
    if(m_PlanningObj->AttachedTo.getStrValue().empty()){
        if(!m_PlanningObj->changeMountState(t_Name.c_str(), true)){
            QMessageBox::warning(NULL,tr("Warning"),tr("Invalid Target Selected, Assemble Failed!"));
        }
    }else{
       m_PlanningObj->changeMountState(t_Name.c_str(), false);
    }
}

void TaskDlgPlanningObject::slot_changeMountPose() {
  double C_x, C_y, C_z, R_r, R_p, R_y;
  C_x = m_ui->doubleSpinBox_Pose_tX->value();
  C_y = m_ui->doubleSpinBox_Pose_tY->value();
  C_z = m_ui->doubleSpinBox_Pose_tZ->value();
  R_r = m_ui->doubleSpinBox_Pose_rY->value();
  R_p = m_ui->doubleSpinBox_Pose_rX->value();
  R_y = m_ui->doubleSpinBox_Pose_rZ->value();
  Base::Placement newPose;
  newPose.setPosition(Vector3d(C_x, C_y, C_z));
  Base::Rotation newRot;
  newRot.setYawPitchRoll(R_y, R_p, R_r);
  newPose.setRotation(newRot);
  m_PlanningObj->Trans_O2M.setValue(newPose);
}

void TaskDlgPlanningObject::slot_changeFramePose()
{
    double C_x, C_y, C_z, R_r, R_p, R_y;
    C_x = m_ui->doubleSpinBox_Frame_tX->value();
    C_y = m_ui->doubleSpinBox_Frame_tY->value();
    C_z = m_ui->doubleSpinBox_Frame_tZ->value();
    R_r = m_ui->doubleSpinBox_Frame_rX->value();
    R_p = m_ui->doubleSpinBox_Frame_rY->value();
    R_y = m_ui->doubleSpinBox_Frame_rZ->value();
    Base::Placement newPose;
    newPose.setPosition(Vector3d(C_x, C_y, C_z));
    Base::Rotation newRot;
    newRot.setYawPitchRoll(R_y, R_p, R_r);
    newPose.setRotation(newRot);
    m_PlanningObj->Trans_O2F.setValue(newPose);
}

void TaskDlgPlanningObject::slot_changeFrameStatus()
{
    if(!m_PlanningObj->FrameOn.getValue())
        m_PlanningObj->FrameOn.setValue(true);
    else
        m_PlanningObj->FrameOn.setValue(false);
    updateFramePanelStatus();
}

void TaskDlgPlanningObject::slot_finishEditObject()
{
    destroyDragger();
    Gui::Control().closeDialog();
}

void TaskDlgPlanningObject::slot_editTargetChanged(int c_index)
{
    switch(c_index){
    case 0: // Mount Pose
        updateDraggerPose(m_PlanningObj->getCurrentMountPose());
        break;
    case 1: // Frame Pose
        if(m_PlanningObj->FrameOn.getValue())
            updateDraggerPose(m_PlanningObj->getCurrentFramePose());
        break;
    default:
        break;
    }
}

void TaskDlgPlanningObject::slot_flipAngle_Mount_rX()
{
    auto result = Robot::DS_Utility::flipRotationValue(m_ui->doubleSpinBox_Pose_rX->value());
    m_ui->doubleSpinBox_Pose_rX->setValue(result);
}

void TaskDlgPlanningObject::slot_flipAngle_Mount_rY()
{
    auto result = Robot::DS_Utility::flipRotationValue(m_ui->doubleSpinBox_Pose_rY->value());
    m_ui->doubleSpinBox_Pose_rY->setValue(result);
}

void TaskDlgPlanningObject::slot_flipAngle_Mount_rZ()
{
    auto result = Robot::DS_Utility::flipRotationValue(m_ui->doubleSpinBox_Pose_rZ->value());
    m_ui->doubleSpinBox_Pose_rZ->setValue(result);
}

void TaskDlgPlanningObject::slot_flipAngle_Frame_rX()
{
    auto result = Robot::DS_Utility::flipRotationValue(m_ui->doubleSpinBox_Frame_rX->value());
    m_ui->doubleSpinBox_Frame_rX->setValue(result);
}

void TaskDlgPlanningObject::slot_flipAngle_Frame_rY()
{
    auto result = Robot::DS_Utility::flipRotationValue(m_ui->doubleSpinBox_Frame_rY->value());
    m_ui->doubleSpinBox_Frame_rY->setValue(result);
}

void TaskDlgPlanningObject::slot_flipAngle_Frame_rZ()
{
    auto result = Robot::DS_Utility::flipRotationValue(m_ui->doubleSpinBox_Frame_rZ->value());
    m_ui->doubleSpinBox_Frame_rZ->setValue(result);

}

bool TaskDlgPlanningObject::createDragger(const Base::Placement &init_Pose)
{
    if(m_PlanningObj_VP == nullptr || m_displayDragger != nullptr)
        return false;
    m_displayDragger = std::make_shared<InteractiveDragger>();
    m_displayDragger->createDragger(m_PlanningObj_VP->getRoot(),
                                    init_Pose,
                                    DraggerUsage::Display);
    m_displayDragger->setAttachingViewProvider(m_PlanningObj_VP);
    return true;
}

void TaskDlgPlanningObject::updateDraggerPose(const Base::Placement new_Pose)
{
    if (m_displayDragger != nullptr)
      m_displayDragger->setDraggerPosition(new_Pose);
    Gui::Command::openCommand("Fit View");
    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
    Gui::Command::commitCommand();

}

void TaskDlgPlanningObject::updateFramePanelStatus()
{
    if(!m_PlanningObj->FrameOn.getValue()){
        m_ui->groupBox_FramePose->setEnabled(false);
        m_ui->pushButton_FrameSwitch->setText(tr("Frame On"));
    }else{
        m_ui->groupBox_FramePose->setEnabled(true);
        m_ui->pushButton_FrameSwitch->setText(tr("Frame Off"));
        updateFramePoseInformation();
    }
}

void TaskDlgPlanningObject::destroyDragger()
{
    if (m_displayDragger == nullptr)
      return;
    m_displayDragger->destroyDragger();
}

#include "moc_TaskDlgPlanningObject.cpp"
