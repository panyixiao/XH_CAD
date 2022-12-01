// Created by Yixiao 2022-05-10

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif
#include "TaskBox_ActionManager.h"
#include <App/DocumentObject.h>
#include <App/GeoFeature.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Document.h>
#include <Gui/ViewProviderDocumentObject.h>
#include <Mod/Part/App/PartFeature.h>
#include <QDoubleSpinBox>
#include <QObject>
#include <qcombobox.h>

using namespace RobotGui;

TaskBox_ActionManager::TaskBox_ActionManager(Robot::ActionObject *t_Action, QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"), tr("New Action"),
              true, parent) {
  if (t_Action == nullptr)
    return;
//  m_DocPtr = pDoc;
  initManagerPanel_Ui();
}

void TaskBox_ActionManager::initManagerPanel_Ui() {
//  m_ui = new Ui_TaskBox_ActionManager;
//  m_proxy_ActionManager = new QWidget();
//  m_ui->setupUi(m_proxy_ActionManager);
//  this->groupLayout()->addWidget(m_proxy_ActionManager);
//  QObject::connect(m_ui->radioButton_RobotTrac,SIGNAL(toggled(bool)),
//                   this,SLOT(changedActionType()));
//  QObject::connect(m_ui->radioButton_Grasp,SIGNAL(toggled(bool)),
//                   this,SLOT(changedActionType()));
//  QObject::connect(m_ui->radioButton_Linear,SIGNAL(toggled(bool)),
//                   this,SLOT(changedActionType()));
//  QObject::connect(m_ui->pushButton_InsertAnAction,
//                   SIGNAL(clicked(bool)), this, SLOT(insertNewAction()));
}
void TaskBox_ActionManager::insertNewAction() {
//  m_proxy_ActionManager->hide();
//  string actionName;
//  switch (m_CurrentTask) {
//  case EditingMode::Create_LinearAction: {
//    actionName = m_DocPtr->getUniqueObjectName("Linear Motion");
//    Gui::Command::openCommand("Insert LinearAction");
//    Gui::Command::doCommand(Gui::Command::Doc, "App.activeDocument().addObject("
//                                               "\"RD_TaskManager::RD_Action_"
//                                               "LinearMovement\",\"%s\")",
//                            actionName.c_str());
//    Gui::Command::commitCommand();
//    auto linearAction = m_DocPtr->getObject(actionName.c_str());
//    if (linearAction)
//      Signal_insertNewLinearAction(linearAction);
//    break;
//  }
//  case EditingMode::Create_GraspingAction: {
//    actionName = m_DocPtr->getUniqueObjectName("Grab");
//    Gui::Command::openCommand("Insert GraspingAction");
//    Gui::Command::doCommand(Gui::Command::Doc, "App.activeDocument().addObject("
//                                               "\"RD_TaskManager::RD_Action_"
//                                               "Grasping\",\"%s\")",
//                            actionName.c_str());
//    Gui::Command::commitCommand();
//    auto graspAction = m_DocPtr->getObject(actionName.c_str());
//    if (graspAction)
//      Signal_insertNewGraspingAction(graspAction);
//    break;
//  }
//  case EditingMode::Create_RobotTrac: {
//      auto objName = m_DocPtr->getUniqueObjectName("Trac");
//      Gui::Command::openCommand("Insert Robot Trac Object");
//      Gui::Command::doCommand(Gui::Command::Doc,
//                              "App.activeDocument().addObject("
//                              "\"RD_TaskManager::RD_TrajectoryObject\",\"%s\")",
//                              objName.c_str());
//      Gui::Command::commitCommand();
//      auto tracAction = m_DocPtr->getObject(objName.c_str());
//      if(tracAction)
//          Signal_insertNewTracAction(tracAction);
//      break;
//  }
//  }
}

void TaskBox_ActionManager::changedActionType() {

//    if(m_ui->radioButton_Grasp->isChecked()){
//     m_CurrentTask = EditingMode::Create_GraspingAction;
//    }
//    else if(m_ui->radioButton_Linear->isChecked()){
//     m_CurrentTask = EditingMode::Create_LinearAction;
//    }
//    else if(m_ui->radioButton_RobotTrac->isChecked()){
//     m_CurrentTask = EditingMode::Create_RobotTrac;
//    }
}
bool TaskBox_ActionManager::accept() { return true; }
bool TaskBox_ActionManager::reject() { return true; }

#include "moc_TaskBox_ActionManager.cpp"
