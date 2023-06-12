// Created By Yixiao 2023-06-06

#ifndef TASKBOX_TRACEDITOR_H
#define TASKBOX_TRACEDITOR_H

#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>
#include <QStringList>
#include <QStringListModel>

#include <Gui/DockWindowManager.h>
#include <Mod/Robot/App/TaskManage/TaskObject.h>

class Ui_TaskBox_TracEditPanel;

using namespace Robot;

namespace RobotGui {
class TaskBox_TracEditPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_TracEditPanel(Robot::TaskObject *t_TaskObjPtr, QWidget *parent = 0);
  bool accept();
  bool reject();
public Q_SLOTS:

private Q_SLOTS:

protected:
  void initUi();

private:
  QWidget *m_proxy;

  Ui_TaskBox_TracEditPanel  *m_ui;

  QStringList *m_ActionList = nullptr;
  QStringListModel *m_ActionListModel = nullptr;

  App::Document *m_currentDoc = nullptr;
  Robot::TaskObject* m_targetTaskPtr = nullptr;

  App::DocumentObject *m_CurrentSelection = nullptr;

};
}

#endif // TASKBOX_TASKMANAGER_H
