// Created By Yixiao 2022-05-10

#ifndef TASKBOX_TASKMANAGER_H
#define TASKBOX_TASKMANAGER_H

#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>
#include <QStringList>
#include <QStringListModel>
#include <QItemSelection>

#include <Gui/DockWindowManager.h>
#include <Mod/Robot/App/TaskManage/TaskObject.h>

//#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/App/Mechanics/MechanicBase.h>
//#include <Mod/Robot/App/Mechanics/MechanicGroup.h>

class Ui_TaskBox_TaskManager;
//class Ui_TaskBox_TaskEditor;

using namespace Robot;
//class Robot6AxisObject;

namespace RobotGui {
class TaskBox_TaskManager : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_TaskManager(Robot::TaskObject *t_TaskObjPtr, QWidget *parent = 0);
  void updateActionList();
  Q_SIGNAL void Signal_insertFreerunTrac(QString);
  Q_SIGNAL void Signal_insertScanTrac(QString);
  Q_SIGNAL void Signal_insertSeamTrac(QString);
  Q_SIGNAL void Signal_editSelectedAction(App::DocumentObject *);
  Q_SIGNAL void Signal_ActionListChanged(Robot::TaskObject*);
  Q_SIGNAL void Signal_insertTracFromFile(QString);

public Q_SLOTS:
  bool accept();
  bool reject();

private Q_SLOTS:
  void slot_insertAction_FreerunTrac();
  void slot_insertAction_ScanTrac();
  void slot_insertAction_SeamTrac();
  void slot_insertAction_ReadinTrac();
  void slot_editSelectedAction();
  void slot_deleteSeletectedAction();
  void slot_moveUpSelectedAction();
  void slot_moveDownSelectedAction();
  void slot_selectionItemChanged(const QItemSelection &selection);
  void slot_exportProgram();

protected:
  void initUi_PanelWidget();
  void udpateOperatorList();
  App::DocumentObject *getSelectedItem(const QItemSelection &selection) const;

private:
  QWidget *m_proxy;
  Ui_TaskBox_TaskManager *m_ui;
  QStringList *m_ActionList = nullptr;
  QStringListModel *m_ActionListModel = nullptr;

  App::Document *m_currentDoc = nullptr;
  Robot::TaskObject* m_targetTaskPtr = nullptr;
  Robot::MechanicBase* m_Executor = nullptr;

  App::DocumentObject *m_CurrentSelection = nullptr;

};
}

#endif // TASKBOX_TASKMANAGER_H
