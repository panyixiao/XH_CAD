#ifndef TASKBOX_ROBOTTRAJECTORYMANAGER_H
#define TASKBOX_ROBOTTRAJECTORYMANAGER_H

#include <App/Document.h>
#include <Gui/TaskView/TaskView.h>
//#include <Mod/RD_Setup/App/rd_RobotObject.h>
//#include <Mod/RD_TaskManager/App/Constraint_PointComposite.h>
//#include <Mod/RD_TaskManager/App/RD_TrajectoryObject.h>
#include <QStringListModel>
#include <QWidget>

//using namespace RD_TaskManager;

//namespace RD_TaskManagerGui {
//class Ui_TaskBox_RobotTrajectoryManager;
//class TaskBox_RobotTrajectoryManager : public Gui::TaskView::TaskBox {
//  Q_OBJECT

//public:
//  explicit TaskBox_RobotTrajectoryManager(App::Document *pDoc,
//                                          RD_TaskManager::RD_TrajectoryObject *t_Trac,
//                                          QWidget *parent = 0);
//  ~TaskBox_RobotTrajectoryManager();
////  void hidePanel();
////  Q_SIGNAL void Signal_tracEditDone(App::DocumentObject *t_Trac);

////private Q_SLOTS:
////  void plan();
////  bool accept();
////  bool reject();
////  void changeConstraintSelection(const QItemSelection &selection);
////  void changeRobotSelection(const QItemSelection &selection);
////  void doneEdit();

////protected:
////  void initUI();
////  void updateConstraintList();
////  void updateRobotList();
////  void updateTrajectoryList();

////private:
////  Ui_TaskBox_RobotTrajectoryManager *m_ui;
////  QWidget *m_proxy;
////  QStringList *m_ConstraintList;
////  QStringListModel *m_ConstraintListModel;
////  QStringList *m_RobotList;
////  QStringListModel *m_RobotListModel;
////  App::Document *m_currentDoc;
////  RD_TaskManager::Constraint_ComposeTask *m_ActiveConstraint = nullptr;
////  RD_Setup::rd_robotObject *m_ActiveRobotPtr = nullptr;
////  RD_TaskManager::RD_TrajectoryObject *m_TracPtr = nullptr;
//};
//}

//#endif // TASKBOX_ROBOTTRAJECTORYMANAGER_H
