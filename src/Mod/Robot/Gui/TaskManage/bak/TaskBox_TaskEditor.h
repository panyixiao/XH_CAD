// Created By Yixiao 2022-05-10

#ifndef TASKBOX_TASKEDITOR_H
#define TASKBOX_TASKEDITOR_H

#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>
#include <QStringList>
#include <QStringListModel>

#include <Gui/DockWindowManager.h>
#include <Mod/Robot/App/TaskManage/TaskObject.h>

#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>

class Ui_TaskBox_TaskEditor;

using namespace Robot;
class Robot6AxisObject;

namespace RobotGui {
class TaskBox_TaskEditor : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_TaskEditor(Robot::TaskObject *t_TaskObjPtr, QWidget *parent = 0);
  bool accept();
  bool reject();
public Q_SLOTS:

private Q_SLOTS:

protected:
  void initUi();

private:
  QWidget *m_proxy;

  Ui_TaskBox_TaskEditor  *m_ui_New;

  QStringList *m_ActionList = nullptr;
  QStringListModel *m_ActionListModel = nullptr;

  App::Document *m_currentDoc = nullptr;
  Robot::TaskObject* m_targetTaskPtr = nullptr;

  Robot::Robot6AxisObject* m_Operator = nullptr;
  App::DocumentObject *m_CurrentSelection = nullptr;

};
}

#endif // TASKBOX_TASKMANAGER_H
