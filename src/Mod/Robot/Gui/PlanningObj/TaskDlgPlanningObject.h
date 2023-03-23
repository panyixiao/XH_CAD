// Created by Yixiao 2017/05/29

#ifndef TASKDIALOG_PLANNINGOBJECT_H
#define TASKDIALOG_PLANNINGOBJECT_H

#include <App/Document.h>
#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include <Gui/TaskView/TaskView.h>
#include <QObject>
#include <QWidget>
#include "Mod/Robot/App/PlanningObj/PlanningObject.h"
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"
#include <Mod/Robot/App/Utilites/DS_Utility.h>

class Ui_TaskDlgPlanningObject;

namespace RobotGui {
class ViewProviderPlanningObj;
class TaskDlgPlanningObject : public Gui::TaskView::TaskDialog {
  Q_OBJECT
public:
  TaskDlgPlanningObject(Robot::PlanningObject *t_PlanningObj,
                        QWidget *parent = 0);

public:
  virtual bool accept();
  virtual bool reject();
  virtual QDialogButtonBox::StandardButtons getStandardButtons() const {
      return QDialogButtonBox::NoButton;
  }

protected:
  void initUi();
  void blockMountPosePanelSignal(bool blocking);
  void blockFramePosePanelSignal(bool blocking);
  void updateMountPanelInformation();
  void updateMountTargetDeviceList();
  void updateFramePoseInformation();
  bool createDragger(const Base::Placement& init_Pose);
  void destroyDragger();
  void updateDraggerPose(const Base::Placement new_Pose);
  void updateFramePanelStatus();

private Q_SLOTS:
  void slot_setMountPoseToFeatureCenter();
  void slot_setFramePoseToFeatureCenter();
  void slot_changeMountState();
  void slot_changeMountPose();
  void slot_changeFramePose();
  void slot_changeFrameStatus();
  void slot_finishEditObject();
  void slot_editTargetChanged(int c_index);
  void slot_flipAngle_Mount_rX();
  void slot_flipAngle_Mount_rY();
  void slot_flipAngle_Mount_rZ();
  void slot_flipAngle_Frame_rX();
  void slot_flipAngle_Frame_rY();
  void slot_flipAngle_Frame_rZ();

private:
  App::Document* m_Doc = nullptr;
  Robot::PlanningObject *m_PlanningObj = nullptr;
  QWidget *m_proxy;
  std::shared_ptr<Ui_TaskDlgPlanningObject> m_ui;
  ViewProviderPlanningObj* m_PlanningObj_VP = nullptr;
  std::shared_ptr<InteractiveDragger> m_displayDragger;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeSelection = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_FaceSelection = nullptr;
};
}

#endif // TASKDIALOG_RD_PLANNINGOBJECT_H
