// Created By Yixiao 2022-05-10

#ifndef TASKBOX_EDGEBASEDTRACEDITOR_H
#define TASKBOX_EDGEBASEDTRACEDITOR_H

#include <QStringList>
#include <QStringListModel>
#include <QWidget>
#include <memory>

#include <App/Document.h>
#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include <Gui/TaskView/TaskView.h>

#include <Mod/Robot/Gui/Utilites/DraggerUtility.h>

#include <Mod/Robot/App/Mechanics/MechanicRobot.h>
#include <Mod/Robot/Gui/Mechanics/ViewProviderMechanicRobot.h>
#include <Mod/Robot/App/Trac/EdgebasedTracObject.h>

class Ui_TaskBox_EdgeBasedTracEditor;
class Ui_TaskBox_EdgeTracGenerator;

namespace RobotGui {
class TaskBox_EdgeBasedTracEditor : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_EdgeBasedTracEditor(Robot::RobotTracObject* t_TracObject,
                              QWidget *parent = 0);
  ~TaskBox_EdgeBasedTracEditor();
  bool initEdgeTracEditor(Robot::RobotTracObject* t_TracObjPtr);
  bool setCurrentSettingPointPoseToNewPose(const Base::Placement &newPose);
  void setPoseAdjustmentToCurrentEdgeWaypoint(const Base::Rotation & pose_Adj);
  void setPoseAdjustmentToRemainEdgeWaypoints(const Base::Rotation& pose_Adj, size_t startPointID);

  Q_SIGNAL void Signal_TracObjectEditDone(App::DocumentObject *obj);
  Q_SIGNAL void Signal_updateTracObject(const RobotProg_sptr);

private Q_SLOTS:
  void slot_finishEdit();
  void slot_generateConstraint();
  // Pose Adjustmen
  void slot_updatePoseAdjustment();
  void slot_applyPoseAdjustmentToRest();
  void slot_updateDiffByCurrentTipPose(bool checked);
  void slot_changeTargetRobotID();
  void slot_changeEdgeDirection(bool flag);
  void slot_flipReference_1(bool flag);
  void slot_flipReference_2(bool flag);
  void slot_changeEdgeInterpoLen(double newStp);
  void slot_sliderPositionChanged(int position);
  void slot_cutRestEdgeFromCurrentPoint();
  void slot_cutFormerEdgeFromCurrentPoint();
  void slot_visualizerChanged();
  // Crafts
  void slot_setTracScanSpeed(double n_speed);
  void slot_setTracWeldSpeed(double n_speed);

protected:

  void initUI_EditEdgePanel();
  void initUI_Generationbox();
  void initUI_PoseEditbox();

  void initUI_WeldParamTab();
  void initUI_ScanParamTab();

  void changeEvent(QEvent *e);
  // Edit Edge
  void enableRobotVisualizer(bool enable, bool restorePose = true);
  void enableMarkerVisualizer(bool enable);
  void updateEdgePointNum();
  void updateEdgeInterpo();
  void updateCurrentPntPose();
  void updateReferenceSelection();
  void setupDragger(bool enable);

  void updateVisualizer();
  void updateDraggerPose(const Base::Placement& new_Pose);
  void updatePoseAdjustmentPanel(const Base::Placement& ajst_Pose);
  void updateRobotPose(const Base::Placement& new_Pose);

  void blockPoseEditboxSignal(bool block);

private:
  Ui_TaskBox_EdgeBasedTracEditor *m_ui = nullptr;
  QWidget *m_proxy;
  App::Document *m_DocPtr;
  Robot::EdgebasedTracObject* m_EdgeTracPtr = nullptr;
  RobotGui::InteractiveDragger *m_MarkerViz = nullptr;

//  uint m_RobotID = 1;
  uint c_PntID = 0;
  // Visualizer
  bool flag_VizByMarker = true;

  Robot::MechanicRobot* m_TracExecutor = nullptr;
  Robot::GroupPose m_InitPoses;
  bool flag_VizByRobot = false;

  Base::Placement m_PntOriginPose;
  std::vector<Base::Placement> m_PntOffsets;

  Gui::TaskView::TaskSelectLinkProperty *m_EdgeSelection = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_FaceSelection = nullptr;
};
}
#endif // EDGEPARAMETER_H
