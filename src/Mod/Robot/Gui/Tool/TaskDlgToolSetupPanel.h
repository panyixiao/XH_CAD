// Created by Yixiao 2022/04/24
#ifndef TASKDIALOG_TOOLSETUP_H
#define TASKDIALOG_TOOLSETUP_H

#include <App/Document.h>
#include <QObject>
#include <QWidget>
#include <functional>
#include <Base/Placement.h>
#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>
#include "Mod/Robot/App/Tool/ToolObject.h"
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

#include "TaskBoxTorchToolSetupPanel.h"
#include "TaskBoxLaserScannerSetupPanel.h"

using namespace Gui;

namespace RobotGui {
class ViewProviderToolObject;
class TaskDlgToolObject : public Gui::TaskView::TaskDialog {
  Q_OBJECT
public:
  TaskDlgToolObject(Robot::ToolObject *targetTool,
                    QWidget *parent = 0);
  QDialogButtonBox::StandardButtons getStandardButtons() const {
//    return QDialogButtonBox::Ok | QDialogButtonBox::Cancel;
      return QDialogButtonBox::NoButton;
  }

private Q_SLOTS:
  bool accept();
  bool reject();
  void slot_updateDraggerPose(const Base::Placement newPlacement);
  // Dragger
  bool createDragger(const Base::Placement& init_Pose);
  void destroyDragger();
  bool draggerIsMoving() {
    if (m_displayDragger != nullptr)
      return m_displayDragger->draggerIsMoving();
    return false;
  }

private:
  App::Document *m_currentDoc = nullptr;
  ViewProviderToolObject *m_toolObj_VP = nullptr;
  std::shared_ptr<InteractiveDragger> m_displayDragger;
};
}

#endif // TASKDIALOG_RD_TOOL_H
