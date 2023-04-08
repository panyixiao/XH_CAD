// Created by Yixiao 2022/05/10

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBox_TaskManager.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/Document.h>

#include <Mod/Robot/Gui/ui_TaskBox_TaskManager.h>

using namespace RobotGui;

TaskBox_TaskManager::TaskBox_TaskManager(Robot::TaskObject *taskObjectPtr,
                                         QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("任务管理"), true, parent) {
  if (taskObjectPtr == nullptr)
    return;
  m_currentDoc = taskObjectPtr->getDocument();
  m_targetTaskPtr = taskObjectPtr;
  initUi_PanelWidget();
}

bool TaskBox_TaskManager::accept() {
//    if(m_Executor != nullptr){
//        m_Executor->Activated.setValue(false);
//    }
    auto act_list = m_targetTaskPtr->getActionList();
    if(act_list.empty() && m_currentDoc!=nullptr)
        m_currentDoc->removeObject(m_targetTaskPtr->getNameInDocument());
    return true;
}

bool TaskBox_TaskManager::reject() {
//    if(m_Executor != nullptr){
//        m_Executor->Activated.setValue(false);
//    }
    auto act_list = m_targetTaskPtr->getActionList();
    if(act_list.empty()&& m_currentDoc!=nullptr)
        m_currentDoc->removeObject(m_targetTaskPtr->getNameInDocument());
    return true;
}


void TaskBox_TaskManager::initUi_PanelWidget() {
    m_proxy = new QWidget();
    m_ui = new Ui_TaskBox_TaskManager;
    m_ui->setupUi(m_proxy);
  this->groupLayout()->addWidget(m_proxy, Qt::AlignTop);

  // Gui Signals
  QObject::connect(m_ui->pushButton_insertSeamTrac, SIGNAL(clicked()),
                   this, SLOT(slot_insertAction_SeamTrac()));
  QObject::connect(m_ui->pushButton_insertScanTrac, SIGNAL(clicked()),
                   this, SLOT(slot_insertAction_ScanTrac()));
  QObject::connect(m_ui->pushButton_insertFreeTrac, SIGNAL(clicked()),
                   this, SLOT(slot_insertAction_FreerunTrac()));

  QObject::connect(m_ui->pushButton_editSelect, SIGNAL(clicked()),
                   this, SLOT(slot_editSelectedAction()));
  QObject::connect(m_ui->pushButton_deleteSelect, SIGNAL(clicked()),
                   this, SLOT(slot_deleteSeletectedAction()));

  QObject::connect(m_ui->pushButton_MoveUp, SIGNAL(clicked()),
                   this, SLOT(slot_moveUpSelectedAction()));
  QObject::connect(m_ui->pushButton_MoveDown, SIGNAL(clicked()),
                   this, SLOT(slot_moveDownSelectedAction()));

  m_ActionList = new QStringList();
  m_ActionListModel = new QStringListModel(*m_ActionList, NULL);
  m_ui->List_Actions->setModel(m_ActionListModel);
  QItemSelectionModel *selectionModelPtr = m_ui->List_Actions->selectionModel();
  QObject::connect(selectionModelPtr, SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
                   this, SLOT(slot_selectionItemChanged(QItemSelection)));
  updateActionList();
  udpateOperatorList();
}

void TaskBox_TaskManager::updateActionList()
{
    if (m_ActionList == nullptr || m_ActionListModel == nullptr)
        return;
    auto act_list = m_targetTaskPtr->getActionList();
    m_ActionList->clear();
    for (auto const &obj_ptr : act_list) {
        auto qstr_name = QObject::tr(obj_ptr->getNameInDocument());
        if (m_ActionList->indexOf(qstr_name) == -1) {
          m_ActionList->append(qstr_name);
        }
    };
    m_ui->pushButton_MoveUp->setEnabled(act_list.size()>1);
    m_ui->pushButton_MoveDown->setEnabled(act_list.size()>1);

    m_ActionListModel->setStringList(*m_ActionList);
}

void TaskBox_TaskManager::udpateOperatorList()
{
      if (m_currentDoc == nullptr)
        return;
//      m_ui->comboBox_OperatorList->clear();
//      auto t_Executors = m_currentDoc->getObjectsOfType(Robot::MechanicBase::getClassTypeId());
////      if (t_Executors.empty()) {
////          Base::Console().Message("TaskBox_TaskManager: No Valid Operator found in Document\n");
////          return;
////      }
//      for (auto opt_Ptr : t_Executors) {
//          m_ui->comboBox_OperatorList->addItem(tr(opt_Ptr->getNameInDocument()));
//      }
}

void TaskBox_TaskManager::slot_insertAction_FreerunTrac()
{
    Q_EMIT Signal_insertFreerunTrac(tr(m_targetTaskPtr->getNameInDocument()));
}

void TaskBox_TaskManager::slot_insertAction_ReadinTrac()
{
    Q_EMIT Signal_insertTracFromFile(tr(m_targetTaskPtr->getNameInDocument()));
}

void TaskBox_TaskManager::slot_insertAction_ScanTrac()
{
    Q_EMIT Signal_insertScanTrac(tr(m_targetTaskPtr->getNameInDocument()));
}

void TaskBox_TaskManager::slot_insertAction_SeamTrac()
{
    Q_EMIT Signal_insertSeamTrac(tr(m_targetTaskPtr->getNameInDocument()));
}


void TaskBox_TaskManager::slot_editSelectedAction()
{
    if(m_CurrentSelection == nullptr)
        return;
    Q_EMIT Signal_editSelectedAction(m_CurrentSelection);
}

void TaskBox_TaskManager::slot_deleteSeletectedAction()
{
    if(m_CurrentSelection == nullptr)
        return;
    m_targetTaskPtr->removeAction(m_ui->List_Actions->selectionModel()->currentIndex().row());
    updateActionList();
    Q_EMIT Signal_ActionListChanged(m_targetTaskPtr);
}

void TaskBox_TaskManager::slot_moveUpSelectedAction()
{
    m_targetTaskPtr->moveActionUp(std::string(m_CurrentSelection->getNameInDocument()));
    updateActionList();
    m_targetTaskPtr->refreshTracData.setValue(true);
    Q_EMIT Signal_ActionListChanged(m_targetTaskPtr);
}

void TaskBox_TaskManager::slot_moveDownSelectedAction()
{
    m_targetTaskPtr->moveActionDown(std::string(m_CurrentSelection->getNameInDocument()));
    updateActionList();
    m_targetTaskPtr->refreshTracData.setValue(true);
    Q_EMIT Signal_ActionListChanged(m_targetTaskPtr);
}

void TaskBox_TaskManager::slot_selectionItemChanged(const QItemSelection &selection)
{
  assert(m_currentDoc != nullptr);
  if (selection.indexes().isEmpty())
    return;
  auto itemName_QString = selection.indexes().first()
          .data(Qt::DisplayRole)
          .toString();
  auto itemName_string = std::string((const char*)itemName_QString.toLocal8Bit().constData());
  auto itemName_char = itemName_string.c_str();
  m_CurrentSelection = m_currentDoc->getObject(itemName_char);
}

void TaskBox_TaskManager::slot_exportProgram()
{
//    bool flag_split = m_ui->checkBox_splitProgram->isChecked();
//    m_targetTaskPtr->exportTaskProgram("/home/yix/test/",flag_split);
}


#include "moc_TaskBox_TaskManager.cpp"
