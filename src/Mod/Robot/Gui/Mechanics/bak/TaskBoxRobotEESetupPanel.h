// Created by Yixiao 20220426

#ifndef TASKBOXROBOTEESETUPPANEL_H
#define TASKBOXROBOTEESETUPPANEL_H

#include <QDialog>
#include <QGridLayout>
#include <QObject>
#include <QPlainTextEdit>
#include <QSignalMapper>
#include <QSlider>
#include <QWidget>

#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

//using namespace RD;

class Ui_TaskBoxRobotEESetupPanel;

enum class SetTarget{
  TorchBase,
  TorchTip,
  SensorBase,
  SensorCMOS
};

namespace RobotGui {

class TaskBoxRobotEESetupPanel : public Gui::TaskView::TaskBox
{
  Q_OBJECT

public:
  TaskBoxRobotEESetupPanel(Robot::Robot6AxisObject *t_robot,
                           Gui::TaskView::TaskSelectLinkProperty * t_FaceSelection,
                           Gui::TaskView::TaskSelectLinkProperty * t_EdgeSelection);

Q_SIGNALS:
  void Signal_finishSetup();
public Q_SLOTS:
//  bool slot_updateCurrentTCPpanel();

protected:
  virtual void mouseReleaseEvent(QMouseEvent* ev);


private:
  bool initUi();
  bool initUi_TorchBox();
  bool initUi_SensorBox();

  void updateTipCenterPose(const Base::Placement& n_Pose);
  void blockTorchPosePanelSignals(bool block);
  void blockSensorPosePanelSignals(bool block);

  void updatePanel_TorchPanel();
  void updatePanel_SensorPanel();
  void updateDraggerPose(const Base::Placement &t_Pose);

private Q_SLOTS:
  void slot_getSelectedCenter();
  void slot_assembleTool2Robot();
  // Poses
  void slot_updateTorchPose();
  void slot_updateSensorPose();

  // Visualization
  void slot_currentSettingTargetChanged();
  void slot_setLaserOn();
  void slot_changeSensorViz();

  void slot_finishSetup();
  bool accept();
  bool reject();

private:
  App::Document *m_DocPtr;
  QWidget *m_proxy;
  Ui_TaskBoxRobotEESetupPanel *m_ui;
  Robot::Robot6AxisObject * m_targetRobot = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_FaceRef = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeRef = nullptr;
  InteractiveDragger* t_Dragger = nullptr;
  SetTarget m_CurrentTarget = SetTarget::TorchTip;
};
}

#endif // TASKBOX_RD_ROBOT_CONFIGURATIONPANEL_H
