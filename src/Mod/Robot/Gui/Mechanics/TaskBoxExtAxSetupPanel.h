// Created by Yixiao 2023-06-12

#ifndef TASKBOX_MECHANICEXTAXIS_H
#define TASKBOX_MECHANICEXTAXIS_H

#include <App/Document.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include <Mod/Robot/App/Mechanics/MechanicExtAx.h>
#include <Mod/Robot/Gui/Utilites/WidgetUtility.h>
#include <Mod/Robot/Gui/Mechanics/QDialogCalibrationPanel.h>

class Ui_TaskBoxExtAxSetupPanel;

namespace RobotGui
{

class TaskBoxExtAxSetupPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT

public:
  TaskBoxExtAxSetupPanel(Robot::MechanicExtAx *t_ExtAxisPtr);

Q_SIGNALS:
  void Signal_setupRobotTool();

public Q_SLOTS:
    void slot_updateSliderPosition();
    // Teach Control
    void slot_setPositionerToHomePose();
    // Config Box
    void slot_targetConfigJointChanged();
    void slot_changeTargetJointLimits();
    void slot_importCalibData();
    void slot_flipTargetAxisDirection();
    void slot_updatePositionerPosePanel();
    bool accept();
    bool reject();

private:
    JointSliderWidget *getTargetJointSlider(const size_t jntID);
    void initUi_PanelWidgets();
    void initUi_AxisControllerBox();
    void initUi_ConfigurationBox();
    void initUi_PoseSetupBox();

private Q_SLOTS:
    void slot_sliderPositionChanged(int t_Index);
    void slot_setCurrentPoseAsHomePosition();
    void slot_updatePositionerPose();
    void slot_updatePositionerReference();
    void slot_referenceTargetChanged();

private:
    App::Document *m_DocPtr = nullptr;
    Robot::MechanicExtAx* m_ExtAxisPtr = nullptr;
    // UI
    QWidget *m_proxy = nullptr;
    Ui_TaskBoxExtAxSetupPanel *m_ui = nullptr;
    // Joint Map
    std::map<string, double> m_JointStatusMap;
    QGridLayout *m_JointPanel_layout = nullptr;
    QSignalMapper *m_signalmapper = nullptr;
    std::vector<JointSliderWidget *> m_jointSliderVec;
};


}


#endif
