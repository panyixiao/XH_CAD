// Create by Yixiao 2022/05/10

#ifndef TASKDIALOG_TASKMANAGE_H
#define TASKDIALOG_TASKMANAGE_H

//#include <Gui/TaskView/TaskDialog.h>
//#include <Gui/TaskView/TaskView.h>

#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"
#include "Mod/Robot/App/TaskManage/TaskObject.h"
#include "Mod/Robot/App/Trac/RobotTracObject.h"

#include <Gui/TaskManagePanel/TaskManageDialog.h>

#include "TaskBox_TaskManager.h"
#include "TaskBox_FreerunTracEditor.h"
#include "TaskBox_EdgeBasedTracEditor.h"
#include "TaskBox_ActionManager.h"
#include "TaskBox_SimulationManager.h"
#include "TaskBox_TracSimulator.h"

namespace RobotGui {

class TaskDlg_TaskManage : public Gui::TaskManage::TaskManageDialog {
  Q_OBJECT
public:
  TaskDlg_TaskManage(Robot::TaskObject *t_TaskObj, QWidget *parent = 0);
  TaskDlg_TaskManage(Robot::RobotTracObject *t_TracObj, QWidget *parent = 0);

  virtual void accept();
  virtual void reject();   
  virtual const std::string getDialogType() const override;
  Q_SIGNAL void Signal_dlg_accept();
  Q_SIGNAL void Signal_dlg_reject();

public Q_SLOTS:
    void slot_editTargetAction(App::DocumentObject* t_obj);
    void slot_insertImportedTrac(QString operator_Name);
    void slot_insertFreerunTrac(QString operator_Name);
    void slot_insertScanTrac(QString operator_Name);
    void slot_insertSeamTrac(QString operator_Name);
    void slot_tracEditDone(App::DocumentObject* t_obj);
    void slot_disableEditing(bool flag);
    void slot_expandSimulationPanel();
    void slot_foldinSimulationPanel();

private:
    void showTskBox_TaskManage(Robot::TaskObject *t_TaskObj);
    void showTskBox_SimuManage(Robot::TaskObject *t_TaskObj);
    void showTskBox_SimuManage(Robot::RobotTracObject *t_TracObj);
    void showTskBox_TracObject(Robot::RobotTracObject* t_TracObj);
    void showTskBox_ActObject(Robot::ActionObject* t_ActObj);
    void hideTskBoxes(bool hide);
    bool insertTracObject(const std::string operatorName, const Robot::TracType& t_type);

private:
  App::Document* m_DocPtr = nullptr;
  Robot::TaskObject *m_taskObjPtr = nullptr;
  // Task Object
  TaskBox_TaskManager* m_TskBox_TaskManager = nullptr;
  // Free Trac
  TaskBox_FreerunTracEditor* m_TskBox_freerunTracEditor = nullptr;
  // Edge Trac
  TaskBox_EdgeBasedTracEditor* m_TskBox_edgebasedTracEditor = nullptr;
  // Simulator
  TaskBox_TracSimulator* m_TskBox_TracSimulator = nullptr;
};

}
#endif // TASKDIALOG_RD_ROBOT_H
