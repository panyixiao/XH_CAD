// Created by Yixiao 2022/05/10

#ifndef TASKBOX_FREERUNTRACEDITOR_H
#define TASKBOX_FREERUNTRACEDITOR_H

#include <Gui/TaskView/TaskView.h>
//#include <Mod/RD_TaskManager/App/Constraint_PointComposite.h>
#include <QStringList>
#include <QStringListModel>
#include <QItemSelection>

#include "Mod/Robot/App/Trac/RobotTracObject.h"
#include "Mod/Robot/App/Mechanics/MechanicGroup.h"
#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"

class Ui_TaskBox_FreerunTracEditor;

namespace RobotGui {


class TaskBox_FreerunTracEditor : public Gui::TaskView::TaskBox {
  Q_OBJECT
public:
  TaskBox_FreerunTracEditor(Robot::RobotTracObject* t_TracObj,
                            QWidget *parent = 0);
  bool setTargetTracObject(Robot::RobotTracObject* t_TracObj);
  Q_SIGNAL void Signal_TracEditDone(App::DocumentObject *);
  Q_SIGNAL void Signal_updateTracObject(const RobotProg_sptr);

protected:
  void initUi_PanelWidget();
  void initUi_CoordBox();
  void initUi_ToolBox();
  const int getSelectedItemID() const;

  void updateCommandWindow();
  void updatePoseLabel(const RobotWaypoint_sptr t_wpnt);
  void updateGroupPose(const RobotWaypoint_sptr t_wpntPtr);
  void resetOperator(bool activated);
  void updateOperatorName();

private Q_SLOTS:
  // Commands
  bool slot_insertCommand_Move();
  void slot_insertCommand_ChangeTool();
  void slot_insertCommand_OperateTool();
  void slot_insertCommand_Coord();
  void slot_deleteCommand();

  void slot_changeGroupOperator();
  void slot_changeOperatorTool();
  void slot_selectionItemChanged(const QItemSelection &selection);
  void slot_pointTypeChanged();
  void slot_changeCoord();
  void slot_finishEdit();

private:
  QWidget *m_proxy = nullptr;
  QButtonGroup* m_btnGroup = nullptr;
  Ui_TaskBox_FreerunTracEditor *m_ui = nullptr;
  QStringList *m_stringList = nullptr;
  QStringListModel *m_stringListModel = nullptr;

  App::Document *m_DocPtr = nullptr;
  Robot::RobotTracObject* m_FreeTrac = nullptr;
  Robot::MechanicGroup* m_MechGroup = nullptr;
//  Robot::Robot6AxisObject* m_RobotPtr = nullptr;
  Robot::ToolType m_TeachTool = Robot::ToolType::Undefined;
  bool switchOn = false;
};
}

#endif
