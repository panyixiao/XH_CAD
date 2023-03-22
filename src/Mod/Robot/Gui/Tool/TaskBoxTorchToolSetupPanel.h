// Created by Yixiao 20220615

#ifndef TASKBOXTORCHTOOLPANEL_H
#define TASKBOXTORCHTOOLPANEL_H

#include <App/Document.h>
#include <QDialog>
#include <QGridLayout>
#include <QObject>
#include <QPlainTextEdit>
#include <QSignalMapper>
#include <QSlider>
#include <QWidget>
#include <QMessageBox>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskSelectLinkProperty.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"
#include <Mod/Robot/App/Tool/TorchObject.h>

//using namespace RD;

class Ui_TaskBoxTorchToolSetupPanel;

namespace RobotGui {

class TaskBoxTorchToolSetupPanel : public Gui::TaskView::TaskBox
{
  Q_OBJECT

public:
  TaskBoxTorchToolSetupPanel(Robot::ToolObject *t_toolPtr,
                             Gui::TaskView::TaskSelectLinkProperty * t_FaceSelection,
                             Gui::TaskView::TaskSelectLinkProperty * t_EdgeSelection);

Q_SIGNALS:
  void Signal_finishSetup();
  void Signal_updateDraggerPose(const Base::Placement);

protected:
  virtual void mouseReleaseEvent(QMouseEvent* ev);


private:
  void initUi();
  void initUi_PoseBox();
  void initUi_VisualizeBox();
  bool initUi_CommBox();

  void updatePoseBox(const Base::Placement& n_Pose);
  void blockPosePanelSignals(bool block);

  void initUi_AssembleWidgets();
  bool checkIfTorchInfoFullfilled();

private Q_SLOTS:
  void slot_getSelectedCenter();
  void slot_assembleTool2Robot();
  // Poses
  void slot_updatePose();
  void slot_setTatgetChanged();

  // Visualization
  void slot_setsparkOn();
  void slot_changeTorchVisualEffect();
  void slot_finishSetupButtonClicked();


private:
  App::Document *m_DocPtr;
  QWidget *m_proxy;
  Ui_TaskBoxTorchToolSetupPanel *m_ui;
  Gui::TaskView::TaskSelectLinkProperty *m_FaceRef = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeRef = nullptr;
  Robot::TorchObject* m_TorchPtr = nullptr;
  bool creatingMode = false;
};
}

#endif
