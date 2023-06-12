// Create by Yixiao 2022/04/24

#ifndef TASKDIALOG_MECHANICCTR_H
#define TASKDIALOG_MECHANICCTR_H

#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>

#include "Mod/Robot/App/Mechanics/MechanicBase.h"
#include "Mod/Robot/App/Mechanics/MechanicRobot.h"
#include "Mod/Robot/App/Mechanics/MechanicPoser.h"
#include "Mod/Robot/App/Mechanics/MechanicExtAx.h"
#include "TaskBoxRobotTeachPanel.h"
#include "TaskBoxPoserSetupPanel.h"
#include "TaskBoxExtAxSetupPanel.h"

namespace RobotGui {
// class ViewProvider_RD_robot;
class TaskDlgMechanicControl : public Gui::TaskView::TaskDialog {
  Q_OBJECT
public:
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
