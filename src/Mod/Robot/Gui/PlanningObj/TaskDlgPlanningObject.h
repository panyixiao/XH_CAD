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

class Ui_TaskDlgPlanningObject;

namespace RobotGui {
class TaskDlgPlanningObject : public Gui::TaskView::TaskDialog {
  Q_OBJECT
public:
  TaskDlgPlanningObject(Robot::PlanningObject *t_PlanningObj,
                        QWidget *parent = 0);

public:
  virtual bool accept();
  virtual bool reject();
  virtual QDialogButtonBox::StandardButtons getStandardButtons() const {
    return QDialogButtonBox::Ok | QDialogButtonBox::Cancel;
  }

protected:
  void initUi();
  void blockPosePanelSignal(bool blocking);
  void updatePanelInformation();
  void updatePositionerList();
  void enablePanel(bool flag);

private Q_SLOTS:
  void slot_resetObjectOrigin();
  void slot_MountToPositioner();
  void slot_changeAssemblePose();

private:
  App::Document* m_Doc = nullptr;
  Robot::PlanningObject *m_PlanningObj = nullptr;
  QWidget *m_proxy;
  std::shared_ptr<Ui_TaskDlgPlanningObject> m_ui;
  Gui::TaskView::TaskSelectLinkProperty *m_EdgeSelection = nullptr;
  Gui::TaskView::TaskSelectLinkProperty *m_FaceSelection = nullptr;
};
}

#endif // TASKDIALOG_RD_PLANNINGOBJECT_H
