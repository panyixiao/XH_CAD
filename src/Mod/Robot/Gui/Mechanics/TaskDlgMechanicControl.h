// Create by Yixiao 2022/04/24

#ifndef TASKDIALOG_MECHANICCTR_H
#define TASKDIALOG_MECHANICCTR_H

#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>

#include "Mod/Robot/App/Mechanics/MechanicBase.h"
#include "Mod/Robot/App/Mechanics/MechanicRobot.h"
#include "Mod/Robot/App/Mechanics/MechanicPoser.h"
#include "TaskBoxRobotTeachPanel.h"
#include "TaskBoxMechanicDevice.h"
#include "TaskBoxMechanicGroupPanel.h"

namespace RobotGui {
// class ViewProvider_RD_robot;
class TaskDlgMechanicControl : public Gui::TaskView::TaskDialog {
  Q_OBJECT
public:
  TaskDlgMechanicControl(Robot::Robot6AxisObject *targetRobot,
                         QWidget *parent = 0);
  TaskDlgMechanicControl(Robot::MechanicDevice* t_Positioner,
                         QWidget *parent = 0);
  TaskDlgMechanicControl(Robot::MechanicGroup* t_Group,
                         QWidget *parent = 0);
  TaskDlgMechanicControl(Robot::MechanicBase* t_Mechanics,
                         QWidget* parent = 0);
  ~TaskDlgMechanicControl();

  virtual bool accept();
  virtual bool reject();
  void closeDlg();
  virtual QDialogButtonBox::StandardButtons getStandardButtons() const {
    return QDialogButtonBox::Ok | QDialogButtonBox::Cancel;
  }

  Q_SIGNAL void signal_dlg_accept();
  Q_SIGNAL void signal_dlg_reject();
  Q_SIGNAL void signal_updatePanelWidgets();
  Q_SIGNAL void signal_updateTipPanel();
  Q_SIGNAL void signal_stationConnected(bool);


private:
  App::Document* m_DocPtr = nullptr;
};

}
#endif // TASKDIALOG_RD_ROBOT_H
