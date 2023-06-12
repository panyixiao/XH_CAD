// Created by Yixiao 2022-08-15

#ifndef TASKBOX_MECHANICGROUPPANEL_H
#define TASKBOX_MECHANICGROUPPANEL_H

#include <App/Document.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include "Mod/Robot/Gui/Utilites/WidgetUtility.h"
#include "Mod/Robot/App/Mechanics/MechanicGroup.h"

class Ui_TaskBoxMechanicGroupPanel;

namespace RobotGui{


class TaskBoxMechanicGroupPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT

public:
  TaskBoxMechanicGroupPanel(Robot::MechanicGroup *t_Group);

Q_SIGNALS:
  void signal_setupRobotTool();

public Q_SLOTS:
  void slot_updatePanelWidgets();
  void slot_stationConnected(bool flag);
  // Teach Controll
  void slot_changeTeachCoord();
  void slot_changeActivatedTool();
  void slot_setRobotToHomePose();
  void slot_setAcitveRobot();
  void slot_controlTargetChanged(int index);
  // Tool Setup
  void slot_changeToolIndex();
  // TCP Controll
  void slot_changeTargetTipPose();
  void slot_updateTipPosePanel();
  // Group setup
  bool accept();
  bool reject();


private:
  JointSliderWidget *findTargetJointSlider(const int jntID);
  void initUi_Panel();
  void initUi_AxisControllerBox();
  void initUi_TcpControlBox();
  void initUi_BodySetupBox();
  void initUi_CommSetupBox();

  void updateBaseSetupBox();
  void updateRobotList();
  void updateLinkedRobotLabel();
  // Poser Panel
  void updatePoserList();
  void updateLinkedPoserAxisTable();


  void blockSignals_TCPbox(bool b);

private Q_SLOTS:
  void sliderPositionChanged(int t_Index);
  void slot_setCurrentPoseAsHomePosition();
  void slot_linkTargetRobot();
  void slot_bindTargetPoser();
  void slot_resetLinkedRobot1();
  void slot_resetLinkedRobot2();
  void slot_resetLinkedPosers();
  void slot_setDraggerAboveAll();
  void slot_pushButtonConnectClicked();
  void slot_networkAddressChanged();
  void slot_enablePoseUpdating();

private:
  // Target
  App::Document *m_DocPtr = nullptr;
  Robot::MechanicGroup* m_MechGroup = nullptr;
  // UI
  QWidget *m_proxy = nullptr;
  Ui_TaskBoxMechanicGroupPanel *m_ui = nullptr;
  // Joint Map
  map<std::string, double> m_JointStatusMap;
  QSignalMapper *m_signalmapper = nullptr;
  vector<RobotGui::JointSliderWidget *> m_jointSliderVec;
  // Property Catcher
  Gui::TaskView::TaskSelectLinkProperty *m_FaceSelection = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeSelection = nullptr;
};


}


#endif
