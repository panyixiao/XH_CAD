// Created by Yixiao 2022/04/25

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/BitmapFactory.h>

#include "TaskDlgPlanningObject.h"
#include "Mod/Robot/Gui/ui_TaskDlgPlanningObject.h"
#include "Mod/Robot/App/Mechanics/MechanicDevice.h"

using namespace RobotGui;

TaskDlgPlanningObject::TaskDlgPlanningObject(Robot::PlanningObject *t_PlanningObj,
                                             QWidget *parent)
    : m_ui(new Ui_TaskDlgPlanningObject), TaskDialog() {
  if (t_PlanningObj == nullptr)
    return;
  m_Doc = t_PlanningObj->getDocument();
  m_PlanningObj = t_PlanningObj;
  initUi();
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

  updatePositionerList();
  setButtonPosition(ButtonPosition::South);
  QObject::connect(m_ui->pushButton_Assemble,SIGNAL(clicked()),
                   this, SLOT(slot_MountToPositioner()));

  updatePanelInformation();

  QObject::connect(m_ui->doubleSpinBox_Cx, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeAssemblePose()));
  QObject::connect(m_ui->doubleSpinBox_Cy, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeAssemblePose()));
  QObject::connect(m_ui->doubleSpinBox_Cz, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeAssemblePose()));
  QObject::connect(m_ui->doubleSpinBox_Rr, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeAssemblePose()));
  QObject::connect(m_ui->doubleSpinBox_Rp, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeAssemblePose()));
  QObject::connect(m_ui->doubleSpinBox_Ry, SIGNAL(valueChanged(double)), this,
                   SLOT(slot_changeAssemblePose()));
  QObject::connect(m_ui->pushButton_reCapCenter, SIGNAL(clicked(bool)),
                   this, SLOT(slot_resetObjectOrigin()));
}

void TaskDlgPlanningObject::blockPosePanelSignal(bool blocking) {
  m_ui->doubleSpinBox_Cx->blockSignals(blocking);
  m_ui->doubleSpinBox_Cy->blockSignals(blocking);
  m_ui->doubleSpinBox_Cz->blockSignals(blocking);
  m_ui->doubleSpinBox_Rr->blockSignals(blocking);
  m_ui->doubleSpinBox_Rp->blockSignals(blocking);
  m_ui->doubleSpinBox_Ry->blockSignals(blocking);
}

void TaskDlgPlanningObject::updatePanelInformation() {
  if (m_PlanningObj == nullptr)
    return;
  blockPosePanelSignal(true);
  m_ui->doubleSpinBox_Cx->setValue(m_PlanningObj->Translation_O2M.getValue().getPosition().x);
  m_ui->doubleSpinBox_Cy->setValue(m_PlanningObj->Translation_O2M.getValue().getPosition().y);
  m_ui->doubleSpinBox_Cz->setValue(m_PlanningObj->Translation_O2M.getValue().getPosition().z);
  double r_y, r_p, r_r;
  m_PlanningObj->Translation_O2M.getValue().getRotation().getYawPitchRoll(r_y, r_p, r_r);
  m_ui->doubleSpinBox_Rp->setValue(r_p);
  m_ui->doubleSpinBox_Rr->setValue(r_r);
  m_ui->doubleSpinBox_Ry->setValue(r_y);
  blockPosePanelSignal(false);
}

void TaskDlgPlanningObject::updatePositionerList() {
  m_ui->comboBox_Positioner->blockSignals(true);
  m_ui->comboBox_Positioner->clear();
  auto t_ObjectList = m_Doc->getObjectsOfType(Robot::MechanicDevice::getClassTypeId());
  m_ui->pushButton_Assemble->setEnabled(!t_ObjectList.empty());
  for(auto t_obj : t_ObjectList){
      m_ui->comboBox_Positioner->addItem(tr(t_obj->getNameInDocument()));
  }
  m_ui->comboBox_Positioner->blockSignals(false);
}

void TaskDlgPlanningObject::enablePanel(bool flag) {
  m_ui->groupBox->setEnabled(flag);
  m_ui->pushButton_reCapCenter->setEnabled(flag);
}

void TaskDlgPlanningObject::slot_resetObjectOrigin() {
  if (m_EdgeSelection->isSelectionValid()) {
    m_EdgeSelection->sendSelection2Property();
  }
  if (m_FaceSelection->isSelectionValid()) {
    m_FaceSelection->sendSelection2Property();
  }
  if (m_PlanningObj != nullptr) {
    m_PlanningObj->setAssembleCenter_toFeatureCenter();
  }
  updatePanelInformation();
}

void TaskDlgPlanningObject::slot_MountToPositioner()
{
    auto t_Name = m_ui->comboBox_Positioner->currentText().toStdString();
    if(!t_Name.empty()){
        auto t_Positioner = static_cast<Robot::MechanicDevice*>(m_Doc->getObject(t_Name.c_str()));
        if(t_Positioner!=nullptr)
            t_Positioner->loadWorkPiece(m_PlanningObj->getNameInDocument());
    }
}

void TaskDlgPlanningObject::slot_changeAssemblePose() {
  double C_x, C_y, C_z, R_r, R_p, R_y;
  C_x = m_ui->doubleSpinBox_Cx->value();
  C_y = m_ui->doubleSpinBox_Cy->value();
  C_z = m_ui->doubleSpinBox_Cz->value();
  R_r = m_ui->doubleSpinBox_Rr->value();
  R_p = m_ui->doubleSpinBox_Rp->value();
  R_y = m_ui->doubleSpinBox_Ry->value();
  Base::Placement newPose;
  newPose.setPosition(Vector3d(C_x, C_y, C_z));
  Base::Rotation newRot;
  newRot.setYawPitchRoll(R_y, R_p, R_r);
  newPose.setRotation(newRot);
  m_PlanningObj->updateTranslation_Origin2Mount(newPose);
}

#include "moc_TaskDlgPlanningObject.cpp"
