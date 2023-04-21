// Created by Yixiao 20220504

#ifndef TASKBOXROBOTTEACHPANEL_H
#define TASKBOXROBOTTEACHPANEL_H

#include <App/Document.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include "Mod/Robot/Gui/Utilites/WidgetUtility.h"
#include "Mod/Robot/App/Mechanics/MechanicRobot.h"
#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"

class Ui_TaskBoxRobotTeachPanel;

namespace RobotGui{


class TaskBoxRobotTeachPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT

public:
  TaskBoxRobotTeachPanel(Robot::MechanicRobot* t_robot,
                         Gui::TaskView::TaskSelectLinkProperty * t_FaceSelection,
                         Gui::TaskView::TaskSelectLinkProperty * t_EdgeSelection);

Q_SIGNALS:
  void Signal_setupRobotTool();

public Q_SLOTS:
  void slot_updatePanelWidgets();
  // Teach Control
  void slot_changeTeachCoord();
  void slot_changeActivatedTool();
  void slot_setRobotToHomePose();
  void slot_sliderPositionChanged(int t_Index);

  // Poser Panel
  void slot_targetPoserChanged();
  void slot_poserButtonClicked();

  // Tool Setup
  void slot_changeToolIndex();

  // TCP Control
  void slot_changeTargetRobotTipPose();
  void slot_updateTipPosePanel();
  void slot_openToolSetupBox();

  // Base Control
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
  JointSliderWidget *findTargetJointSlider(const size_t jntID);
  void initUi();
  void initUi_AxisControllerBox();
  void initUi_ConfigurationBox();
  void initUi_PoserSetupBox();
  void initUi_TcpControlBox();
  void initUi_BaseSetupBox();

  void updateLinkedPoserWidgets();
  void updateRef2BasePosePanel();

  void blockSignals_TCPbox(bool b);
  void blockSignals_Basebox(bool b);

private Q_SLOTS:
  void slot_setCurrentPoseAsHomePosition();
//  void targetJointChanged();

private:
  // Target
  App::Document *m_DocPtr = nullptr;
  Robot::MechanicRobot* m_RobotPtr = nullptr;
  // UI
  QWidget *m_proxy = nullptr;
  Ui_TaskBoxRobotTeachPanel *m_ui = nullptr;
  // Joint Map
  map<std::string, double> m_JointStatusMap;
  QGridLayout *m_Layout_RobotJointPanel = nullptr;
  QGridLayout *m_Layout_PoserJointPanel = nullptr;
  QGridLayout *m_Layout_ExtAxJointPanel = nullptr;
  QSignalMapper *m_signalmapper = nullptr;
  vector<RobotGui::JointSliderWidget *> m_jointSliderVec;
  // Property Catcher
  Gui::TaskView::TaskSelectLinkProperty *m_FaceSelection = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeSelection = nullptr;

  bool operationFlag_bind = false;
};


}


#endif // TASKBOX_RD_ROBOT_CONFIGURATIONPANEL_H
