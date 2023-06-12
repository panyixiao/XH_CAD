// Created By Yixiao 2023-06-06

#ifndef TASKBOX_FEATUREPATHPANEL_H
#define TASKBOX_FEATUREPATHPANEL_H

#include <Gui/TaskView/TaskDialog.h>
#include <Gui/TaskView/TaskView.h>
#include <QStringList>
#include <QStringListModel>

#include <Gui/DockWindowManager.h>
#include <Mod/Robot/App/TaskManage/TaskObject.h>

class Ui_TaskBox_FeaturePathPanel;

using namespace Robot;

namespace RobotGui {
class TaskBox_FeaturePathPanel : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_FeaturePathPanel(Robot::TaskObject *t_TaskObjPtr, QWidget *parent = 0);
  bool accept();
  bool reject();

protected:
  void initUi();

private:
  QWidget *m_proxy;

  Ui_TaskBox_FeaturePathPanel  *m_ui_New;

  QStringList *m_ActionList = nullptr;
  QStringListModel *m_ActionListModel = nullptr;

  App::Document *m_currentDoc = nullptr;
  Robot::TaskObject* m_targetTaskPtr = nullptr;

//  Robot::Robot6AxisObject* m_Operator = nullptr;
  App::DocumentObject *m_CurrentSelection = nullptr;
};
}

#endif // TASKBOX_TASKMANAGER_H
