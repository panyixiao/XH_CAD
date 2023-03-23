// Created by Yixiao 2022-08-09

#ifndef TASKBOX_WELDINGPOSITIONER_H
#define TASKBOX_WELDINGPOSITIONER_H

#include <App/Document.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include <Mod/Robot/Gui/Utilites/WidgetUtility.h>
#include <Mod/Robot/App/Mechanics/MechanicPoser.h>
#include <Mod/Robot/Gui/Mechanics/QDialogCalibrationPanel.h>

class Ui_TaskBoxPoserSetupPanel;

namespace RobotGui
{

class TaskBoxPoserSetupPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT

public:
  TaskBoxPoserSetupPanel(Robot::MechanicPoser *t_Positioner);

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
    JointSliderWidget *getTargetJointSlider(const int jntID);
    void initUi_PanelWidgets();
    void initUi_AxisControllerBox();
    void initUi_ConfigurationBox();
    void initUi_PoseSetupBox();

private Q_SLOTS:
    void sliderPositionChanged(int t_Index);
    void slot_setCurrentPoseAsHomePosition();
    void slot_updatePositionerPose();
    void slot_updatePositionerReference();
    void slot_referenceTargetChanged();

private:
    App::Document *m_DocPtr = nullptr;
    Robot::MechanicPoser* m_PoserPtr = nullptr;
    // UI
    QWidget *m_proxy = nullptr;
    Ui_TaskBoxPoserSetupPanel *m_ui = nullptr;
    // Joint Map
    std::map<string, double> m_JointStatusMap;
    QGridLayout *m_JointPanel_layout = nullptr;
    QSignalMapper *m_signalmapper = nullptr;
    std::vector<JointSliderWidget *> m_jointSliderVec;
};


}


#endif
