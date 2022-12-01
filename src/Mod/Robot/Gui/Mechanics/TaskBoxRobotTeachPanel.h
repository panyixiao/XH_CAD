// Created by Yixiao 20220504

#ifndef TASKBOXROBOTTEACHPANEL_H
#define TASKBOXROBOTTEACHPANEL_H

#include <App/Document.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include "Mod/Robot/Gui/Utilites/WidgetUtility.h"
#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"

class Ui_TaskBoxRobotTeachPanel;

namespace RobotGui{


class TaskBoxRobotTeachPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT

public:
  TaskBoxRobotTeachPanel(Robot::Robot6AxisObject *t_robot,
                         Gui::TaskView::TaskSelectLinkProperty * t_FaceSelection,
                         Gui::TaskView::TaskSelectLinkProperty * t_EdgeSelection);

Q_SIGNALS:
  void Signal_setupRobotTool();

public Q_SLOTS:
  void slot_updatePanelWidgets();
  // Teach Controll
  void slot_changeTeachCoord();
  void slot_changeActivatedTool();
  void slot_setRobotToHomePose();

  // Tool Setup
  void slot_changeToolIndex();

  // TCP Controll
  void slot_changeTargetRobotTipPose();
  void slot_updateTipPosePanel();
  void slot_openToolSetupBox();

  // Base Controll
  void slot_changeRef2BasePose();
  void slot_changeReferenceBase();

  // Config Box
  void slot_targetConfigJointChanged();
  void slot_changeTargetJointLimits();
  void slot_changeArmConfig();
  void slot_flipAxisDirection();
  void slot_enableConfigConstraint();

  bool accept();
  bool reject();


private:
  JointSliderWidget *findTargetJointSlider(const int jntID);
  void initUi();
  void initUi_AxisControllerBox();
  void initUi_ConfigurationBox();
  void initUi_GroupSetupBox();
  void initUi_TcpControlBox();
  void initUi_BaseSetupBox();
  void updateRef2BasePosePanel();

  void blockSignals_TCPbox(bool b);
  void blockSignals_Basebox(bool b);

private Q_SLOTS:
  void sliderPositionChanged(int t_Index);
  void slot_setCurrentPoseAsHomePosition();
//  void targetJointChanged();

private:
  // Target
  App::Document *m_DocPtr = nullptr;
  Robot::Robot6AxisObject* m_RobotPtr = nullptr;
  // UI
  QWidget *m_proxy = nullptr;
  Ui_TaskBoxRobotTeachPanel *m_ui = nullptr;
  // Joint Map
  map<std::string, double> m_JointStatusMap;
  QGridLayout *m_JointPanel_layout = nullptr;
  QSignalMapper *m_signalmapper = nullptr;
  vector<RobotGui::JointSliderWidget *> m_jointSliderVec;
  // Property Catcher
  Gui::TaskView::TaskSelectLinkProperty *m_FaceSelection = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeSelection = nullptr;
};


}


#endif // TASKBOX_RD_ROBOT_CONFIGURATIONPANEL_H
