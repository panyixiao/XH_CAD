// Created by Yixiao 20220615

#ifndef TASKBOXLASERSCANNERPANEL_H
#define TASKBOXLASERSCANNERPANEL_H

#include <App/Document.h>
#include <QDialog>
#include <QGridLayout>
#include <QObject>
#include <QPlainTextEdit>
#include <QSignalMapper>
#include <QSlider>
#include <QWidget>
#include <Mod/Robot/App/Tool/ScannerObject.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

//using namespace RD;

class Ui_TaskBoxLaserScannerSetupPanel;

namespace RobotGui {

class TaskBoxLaserScannerSetupPanel : public Gui::TaskView::TaskBox
{
  Q_OBJECT

public:
  TaskBoxLaserScannerSetupPanel(Robot::ToolObject *t_toolPtr,
                                Gui::TaskView::TaskSelectLinkProperty * t_FaceSelection,
                                Gui::TaskView::TaskSelectLinkProperty * t_EdgeSelection);

Q_SIGNALS:
  void Signal_finishSetup();
  void Signal_updateDraggerPose(const Base::Placement);

public Q_SLOTS:
//  bool slot_updateCurrentTCPpanel();

protected:
  virtual void mouseReleaseEvent(QMouseEvent* ev);


private:
  void initUi();
  void initUi_PoseBox();
  void initUi_VisualizeBox();
  void initUi_CalibBox();

  void updatePoseBox(const Base::Placement& n_Pose);
  void enablePoseBox(bool flag);
  void blockPosePanelSignals(bool block);

  void initUi_AssembleWidgets();

private Q_SLOTS:
  void slot_getSelectedCenter();
  void slot_assembleTool2Robot();
  // Poses
  void slot_updatePose();
  void slot_setTatgetChanged();

  // Visualization
  void slot_setLaserOn();
  void slot_changeScanRange();

  // Calib
  void slot_calibLenPose();

  void slot_finishSetup();
  bool accept();
  bool reject();

private:
  App::Document *m_DocPtr;
  QWidget *m_proxy;
  Ui_TaskBoxLaserScannerSetupPanel *m_ui;
  Gui::TaskView::TaskSelectLinkProperty *m_FaceRef = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeRef = nullptr;
  Robot::ScannerObject* m_ScannerPtr = nullptr;
};
}

#endif // TASKBOX_RD_ROBOT_CONFIGURATIONPANEL_H
