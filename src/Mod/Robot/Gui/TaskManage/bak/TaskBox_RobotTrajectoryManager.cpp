//#include "PreCompiled.h"
//#ifndef _PreComp_
//#endif
//#include "TaskBox_RobotTrajectoryManager.h"
//#include <App/DocumentObject.h>
//#include <Base/Console.h>
//#include <Gui/BitmapFactory.h>
//#include <Mod/RD_TaskManager/App/Constraint_ComposeTask.h>
//#include <Mod/RD_TaskManager/Gui/ui_TaskBox_RobotTrajectoryManager.h>

//using namespace RD_TaskManagerGui;

//TaskBox_RobotTrajectoryManager::TaskBox_RobotTrajectoryManager(
//    App::Document *pDoc, RD_TrajectoryObject *t_Trac, QWidget *parent)
//    : TaskBox(Gui::BitmapFactory().pixmap("document-new"), tr("Trajectory"),
//              true, parent) {
////  if (pDoc == nullptr || t_Trac == nullptr)
////    return;
////  m_currentDoc = pDoc;
////  m_TracPtr = t_Trac;
////  initUI();
////  updateRobotList();
////  m_ui->listView_RobotList->setEnabled(false);
////  updateConstraintList();
////  updateTrajectoryList();
//}

//TaskBox_RobotTrajectoryManager::~TaskBox_RobotTrajectoryManager() {
////  delete m_ui;
////  delete m_proxy;
//}

////void TaskBox_RobotTrajectoryManager::hidePanel() { m_proxy->hide(); }

////bool TaskBox_RobotTrajectoryManager::accept() {
////  //  if(m_currentDoc == nullptr)
////  //      return false;
////  //  if (m_TracPtr != nullptr && !m_TracPtr->fullyDefined()) {
////  //      m_TracPtr->resetConstraintParents();
////  //      m_currentDoc->removeObject(m_TracPtr->getNameInDocument());
////  //  }
////  return true;
////}

////bool TaskBox_RobotTrajectoryManager::reject() {
////  //  if(m_currentDoc == nullptr)
////  //      return false;
////  //  if (m_TracPtr != nullptr && !m_TracPtr->fullyDefined()) {
////  //    m_TracPtr->resetConstraintParents();
////  //    m_currentDoc->removeObject(m_TracPtr->getNameInDocument());
////  //  }
////  return true;
////}
////void TaskBox_RobotTrajectoryManager::initUI() {
////  m_proxy = new QWidget();
////  m_ui = new Ui_TaskBox_RobotTrajectoryManager;
////  m_ui->setupUi(m_proxy);
////  this->groupLayout()->addWidget(m_proxy);
////  QObject::connect(m_ui->pushButton_Calc, SIGNAL(clicked(bool)),
////                   this,SLOT(plan()));
////  // ListView
////  m_ConstraintList = new QStringList();
////  m_ConstraintListModel = new QStringListModel(*m_ConstraintList, NULL);
////  m_ui->listView_ConstraintList->setModel(m_ConstraintListModel);
////  m_RobotList = new QStringList();
////  m_RobotListModel = new QStringListModel(*m_RobotList, NULL);
////  m_ui->listView_RobotList->setModel(m_RobotListModel);
////  auto constraintSelectionModel =
////      m_ui->listView_ConstraintList->selectionModel();

////  QObject::connect(constraintSelectionModel,
////                   SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
////                   this, SLOT(changeConstraintSelection(QItemSelection)));

////  auto robotSelectionModel = m_ui->listView_RobotList->selectionModel();

////  QObject::connect(robotSelectionModel,
////                   SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
////                   this, SLOT(changeRobotSelection(QItemSelection)));
////  QObject::connect(m_ui->pushButton_Done, SIGNAL(clicked()), this,
////                   SLOT(doneEdit()));
////}
////void TaskBox_RobotTrajectoryManager::updateConstraintList() {
////  assert(m_currentDoc);
////  auto objList = m_currentDoc->getObjects();
////  for (auto objPtr : objList) {
////    if (objPtr->isDerivedFrom(
////            RD_TaskManager::Constraint_ComposeTask::getClassTypeId())) {
////      auto constraint =
////          dynamic_cast<RD_TaskManager::Constraint_ComposeTask *>(objPtr);
////      //      if (strlen(constraint->Parent.getValue()) == 0)
////      m_ConstraintList->append(
////          QString::fromStdString(string(constraint->getNameInDocument())));
////    }
////  }
////  m_ConstraintListModel->setStringList(*m_ConstraintList);
////}

////void TaskBox_RobotTrajectoryManager::updateRobotList() {
////  assert(m_currentDoc);
////  auto obj_List = m_currentDoc->getObjects();
////  for (auto objPtr : obj_List) {
////    if (objPtr->isDerivedFrom(RD_Setup::rd_robotObject::getClassTypeId()))
////      m_RobotList->append(
////          QString::fromStdString(string(objPtr->getNameInDocument())));
////  }
////  m_RobotListModel->setStringList(*m_RobotList);
////}

////void TaskBox_RobotTrajectoryManager::updateTrajectoryList() {
////  //  m_TracList->clear();
////  //  if (m_TracPtr != nullptr) {
////  //    m_TracList->append(QString::fromStdString(string(m_TracPtr->getNameInDocument())));
////  //  }
////  //  m_TracListModel->setStringList(*m_TracList);
////  if (m_TracPtr != nullptr)
////    m_ui->textEdit_TracObject->setText(
////        QString::fromStdString(string(m_TracPtr->getNameInDocument())));
////}

////void TaskBox_RobotTrajectoryManager::changeConstraintSelection(
////    const QItemSelection &selection) {
////  if (selection.indexes().isEmpty())
////    return;
////  auto constrainName = selection.indexes()
////                           .first()
////                           .data(Qt::DisplayRole)
////                           .toString()
////                           .toStdString()
////                           .c_str();
////  auto constraint = m_currentDoc->getObject(constrainName);
////  if (constraint != nullptr) {
////    m_ActiveConstraint =
////        dynamic_cast<RD_TaskManager::Constraint_ComposeTask *>(constraint);
////  }
////}
////void TaskBox_RobotTrajectoryManager::changeRobotSelection(
////    const QItemSelection &selection) {
////  if (selection.indexes().isEmpty())
////    return;
////  auto roboName = selection.indexes()
////                      .first()
////                      .data(Qt::DisplayRole)
////                      .toString()
////                      .toStdString()
////                      .c_str();
////  auto robotPtr = m_currentDoc->getObject(roboName);
////  if (robotPtr != nullptr) {
////    m_ActiveRobotPtr = dynamic_cast<RD_Setup::rd_robotObject *>(robotPtr);
////  }
////}

////void TaskBox_RobotTrajectoryManager::doneEdit() {
////  if (m_TracPtr)
////    Signal_tracEditDone(m_TracPtr);
////  this->hideGroupBox();
////}

////void TaskBox_RobotTrajectoryManager::plan() {
////  if (m_ActiveConstraint == nullptr || m_ActiveRobotPtr == nullptr) {
////    Base::Console().Message("Planning Target not assigned, Planning failed!\n");
////    return;
////  }

////  // Set Constraints
////  vector<PoseStampedConstraint> constraintVec;
////  RD_WayPoint initPos;
////  initPos.setPlacement(m_ActiveRobotPtr->getCurrentTipPosition());
////  for (auto const &tskPnt : m_ActiveConstraint->getWayPointVec()) {
////    constraintVec.push_back(tskPnt->getPlanningConstraint());
////  }
////  bool useMoveit = false;

////  // Planning
////  std::vector<bool> inCollision;
////  std::vector<bool> ikFailed;
////  auto traj = m_ActiveRobotPtr->getLibInterfacePtr()->planByConstraint(
////      constraintVec, inCollision, ikFailed, useMoveit,
////      m_ActiveRobotPtr->getJointConfiguration());
////  if (traj != nullptr) {
////    // Plan result
////    if (ikFailed.size()) {
////      int failedNum = 0;
////      for (auto flag : ikFailed) {
////        if (flag)
////          failedNum++;
////      }
////      auto ratio = 100 * (1 - float(failedNum) / float(ikFailed.size()));
////      Base::Console().Message("Total %2.2f % Way Points Planning Success!\n",
////                              ratio);
////    }
////    m_TracPtr->executeAction(0);
////  }
////}
//#include "TaskManage/moc_TaskBox_RobotTrajectoryManager.cpp"
