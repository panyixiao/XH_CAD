// Created by Yixiao 2022-05-10

#ifndef TASKBOX_ACTIONMANAGER_H
#define TASKBOX_ACTIONMANAGER_H

#include <App/Document.h>
#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>
#include "Mod/Robot/App/TaskManage/ActionObject.h"


class Ui_TaskBox_ActionManager;

namespace RobotGui {
//class Ui_TaskBox_LinearAction;
//class Ui_TaskBox_GraspingAction;

class TaskBox_ActionManager : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_ActionManager(Robot::ActionObject* t_ActionObj,
                        QWidget *parent = 0);
  Q_SIGNAL void Signal_removeTargetActionFromList(char* actName );

//  Q_SIGNAL void Signal_insertNewLinearAction(App::DocumentObject *t_Action);
//  Q_SIGNAL void Signal_insertNewGraspingAction(App::DocumentObject *t_Grasp);
//  Q_SIGNAL void Signal_insertNewTracAction(App::DocumentObject *t_trac);

protected:
  void initManagerPanel_Ui();

private Q_SLOTS:
  void changedActionType();
  void insertNewAction();
  bool accept();
  bool reject();

private:
  // Ui
  Ui_TaskBox_ActionManager *m_ui = nullptr;
  QWidget *m_proxy_ActionManager = nullptr;
//  App::Document *m_DocPtr = nullptr;
//  EditingMode m_CurrentTask;
};
}

#endif
