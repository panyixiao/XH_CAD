// Created By Yixiao 2022-04-24

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <thread>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>
#include <Gui/Command.h>
#include <map>

#include "QGroupBox"
#include "QLineEdit"
#include "QSignalMapper"
#include "QFileDialog"
#include "TaskDlg_TaskManage.h"

#include "Mod/Robot/App/Trac/RobotTracObject.h"

#define Maxium_Width 420

using namespace RobotGui;

TaskDlg_TaskManage::TaskDlg_TaskManage(Robot::TaskObject *t_TaskObj,
                                       QWidget *parent): TaskManageDialog(parent) {
  if (t_TaskObj == nullptr)
    return;
  m_taskObjPtr = t_TaskObj;
  m_DocPtr = t_TaskObj->getDocument();

  showTskBox_TaskManage(t_TaskObj);
  showTskBox_SimuManage(t_TaskObj);
  this->setMaximumWidth(Maxium_Width+5);
}

TaskDlg_TaskManage::TaskDlg_TaskManage(RobotTracObject *t_TracObj, QWidget *parent)
{
    if(t_TracObj == nullptr)
        return;
    m_taskObjPtr = static_cast<Robot::TaskObject*>(t_TracObj->getTracManager());
    m_DocPtr = t_TracObj->getDocument();

    if(m_TskBox_TracSimulator == nullptr){
        m_TskBox_TracSimulator = new TaskBox_TracSimulator(t_TracObj);
    }
    showTskBox_TracObject(t_TracObj);
    showTskBox_SimuManage(t_TracObj);
    this->setMaximumWidth(Maxium_Width+5);
}

void TaskDlg_TaskManage::accept() {
    Q_EMIT Signal_dlg_accept();
    Gui::Document *document = Gui::Application::Instance->getDocument(m_DocPtr);
    if (document != nullptr) {
      document->commitCommand();
      document->resetEdit();
    }
    this->done(QDialog::Accepted);
}

void TaskDlg_TaskManage::reject() {
    Gui::Document *document = Gui::Application::Instance->getDocument(m_DocPtr);
    if (document != nullptr) {
      document->commitCommand();
      document->resetEdit();
    }
    this->done(QDialog::Rejected);
}

void TaskDlg_TaskManage::slot_insertFreerunTrac(QString operator_Name){
    insertTracObject(operator_Name.toStdString(), Robot::TracType::FREETRAC);
    hideTskBoxes(true);
}

void TaskDlg_TaskManage::slot_insertScanTrac(QString operator_Name)
{
    insertTracObject(operator_Name.toStdString(), Robot::TracType::SCANTRAC);
    hideTskBoxes(true);
}

void TaskDlg_TaskManage::slot_insertSeamTrac(QString operator_Name)
{
    insertTracObject(operator_Name.toStdString(), Robot::TracType::SEAMTRAC);
    hideTskBoxes(true);
}

void TaskDlg_TaskManage::slot_tracEditDone(App::DocumentObject * t_obj)
{
    if(m_TskBox_freerunTracEditor)
        m_TskBox_freerunTracEditor = nullptr;
    if(m_TskBox_edgebasedTracEditor)
        m_TskBox_edgebasedTracEditor = nullptr;

    // This happens when Edit Panel created by doubleclick TracObject
    if(m_taskObjPtr == nullptr){
        if(t_obj == nullptr)
            return;
        if(t_obj->isDerivedFrom(Robot::RobotTracObject::getClassTypeId())){
            auto trac_obj = static_cast<Robot::RobotTracObject*>(t_obj);
            if(trac_obj==nullptr)
                return;
            // Get TracObject Task Name
            m_taskObjPtr = static_cast<Robot::TaskObject*>(trac_obj->getDocument()->getObject(trac_obj->TracManagerName.getValue()));
        }
    }
    m_taskObjPtr->refreshTracData.setValue(true);
    showTskBox_TaskManage(m_taskObjPtr);
    showTskBox_SimuManage(m_taskObjPtr);
    m_DlgCtrl->showButtons(true);
}

void TaskDlg_TaskManage::slot_disableEditing(bool flag)
{
    if(m_TskBox_TaskManager!=nullptr)
        m_TskBox_TaskManager->setEnabled(!flag);
    if(m_TskBox_freerunTracEditor!=nullptr)
        m_TskBox_freerunTracEditor->setEnabled(!flag);
    if(m_TskBox_edgebasedTracEditor!= nullptr)
        m_TskBox_edgebasedTracEditor->setEnabled(!flag);

}

void TaskDlg_TaskManage::slot_expandSimulationPanel()
{
    if(!m_TskBox_TracSimulator)
        return;
    m_TskBox_TracSimulator->setMinimumHeight(850);
    if(m_TskBox_freerunTracEditor)
        m_TskBox_freerunTracEditor->hideGroupBox();
    if(m_TskBox_edgebasedTracEditor)
        m_TskBox_edgebasedTracEditor->hideGroupBox();
    if(m_TskBox_TaskManager)
        m_TskBox_TaskManager->hideGroupBox();
    m_DlgCtrl->hide();
}

void TaskDlg_TaskManage::slot_foldinSimulationPanel()
{
    if(!m_TskBox_TracSimulator)
        return;
    m_TskBox_TracSimulator->setMinimumHeight(220);
    if(m_TskBox_freerunTracEditor)
        m_TskBox_freerunTracEditor->showHide();
    if(m_TskBox_edgebasedTracEditor)
        m_TskBox_edgebasedTracEditor->showHide();
    if(m_TskBox_TaskManager)
        m_TskBox_TaskManager->showHide();
    m_DlgCtrl->show();
}

void TaskDlg_TaskManage::slot_editTargetAction(App::DocumentObject *t_obj)
{
    if(t_obj == nullptr)
        return;
    if(t_obj->isDerivedFrom(Robot::RobotTracObject::getClassTypeId())){
        showTskBox_TracObject(static_cast<Robot::RobotTracObject*>(t_obj));
        showTskBox_SimuManage(static_cast<Robot::RobotTracObject*>(t_obj));
    }
    else if(t_obj->isDerivedFrom(Robot::ActionObject::getClassTypeId())){
        showTskBox_ActObject(static_cast<Robot::ActionObject*>(t_obj));
    }
    hideTskBoxes(true);
}

void TaskDlg_TaskManage::slot_insertImportedTrac(QString operator_Name)
{
    Gui::Command::openCommand("Import Trac From File");
    std::string tracName = m_DocPtr->getUniqueObjectName("SavedTrac");
    std::string cmmd_str = "App.activeDocument().addObject(\"Robot::RobotTracObject\",\"%s\")";
    Gui::Command::doCommand(Gui::Command::Doc, cmmd_str.c_str(),
                            tracName.c_str());
    Gui::Command::commitCommand();
    // Get inserted Object
    auto objPtr = m_DocPtr->getObject(tracName.c_str());
    if(objPtr != nullptr){
        QString fileName = QFileDialog::getOpenFileName(NULL,tr("Target Trac Selection"),tr("/home"),tr("program(*.prg *.txt)"));
        if(fileName.isEmpty()){
            Base::Console().Message("No Trac File Selected!\n");
            m_DocPtr->removeObject(tracName.c_str());
            return;
        }

        auto t_ProgramPtr = FileIO_Utility::readinProgramFromDrive(fileName.toStdString(),
                                                                   operator_Name.toStdString());

        // Set Trac operator
        auto tracObj = static_cast<Robot::RobotTracObject*>(objPtr);
        tracObj->setTracManager(std::string(m_taskObjPtr->getNameInDocument()));
        tracObj->setOperator(operator_Name.toStdString());
        tracObj->setTracType(Robot::TracType::FREETRAC);
        tracObj->setRobotProgramPtr(t_ProgramPtr);
        m_taskObjPtr->insertAction(tracObj);
        showTskBox_TracObject(tracObj);
        hideTskBoxes(true);
    }
}

void TaskDlg_TaskManage::showTskBox_TaskManage(TaskObject *t_TaskObj)
{
    if(t_TaskObj == nullptr)
        return;
    if(m_TskBox_TaskManager == nullptr){
        m_TskBox_TaskManager = new TaskBox_TaskManager(t_TaskObj);
        QObject::connect(this, SIGNAL(Signal_dlg_accept()),
                         m_TskBox_TaskManager, SLOT(accept()));
        QObject::connect(this, SIGNAL(Signal_dlg_reject()),
                         m_TskBox_TaskManager, SLOT(reject()));

        QObject::connect(m_TskBox_TaskManager, SIGNAL(Signal_insertFreerunTrac(QString)),
                         this, SLOT(slot_insertFreerunTrac(QString)));
        QObject::connect(m_TskBox_TaskManager, SIGNAL(Signal_insertScanTrac(QString)),
                         this, SLOT(slot_insertScanTrac(QString)));
        QObject::connect(m_TskBox_TaskManager, SIGNAL(Signal_insertSeamTrac(QString)),
                         this, SLOT(slot_insertSeamTrac(QString)));
        QObject::connect(m_TskBox_TaskManager, SIGNAL(Signal_insertTracFromFile(QString)),
                         this, SLOT(slot_insertImportedTrac(QString)));
        QObject::connect(m_TskBox_TaskManager, SIGNAL(Signal_editSelectedAction(App::DocumentObject*)),
                         this, SLOT(slot_editTargetAction(App::DocumentObject*)));
        addWidget(m_TskBox_TaskManager);
        if(m_TskBox_TracSimulator)
            QObject::connect(m_TskBox_TaskManager, SIGNAL(Signal_ActionListChanged(Robot::TaskObject*)),
                             m_TskBox_TracSimulator, SLOT(slot_updateSimProgram(Robot::TaskObject*)));
    }
    else{
        m_TskBox_TaskManager->updateActionList();
        m_TskBox_TaskManager->show();
    }
    m_TskBox_TaskManager->setMaximumWidth(Maxium_Width);
    this->resize(m_TskBox_TaskManager->width(),this->height());
}

void TaskDlg_TaskManage::showTskBox_SimuManage(TaskObject *t_TaskObj)
{
    if(t_TaskObj == nullptr)
        return;
    if(m_TskBox_TracSimulator == nullptr){
        m_TskBox_TracSimulator = new TaskBox_TracSimulator(t_TaskObj);
        m_TskBox_TracSimulator->setMaximumWidth(Maxium_Width);
        QObject::connect(m_TskBox_TracSimulator, SIGNAL(Signal_simulationOn(bool)),
                         this, SLOT(slot_disableEditing(bool)));
        QObject::connect(m_TskBox_TracSimulator, SIGNAL(Signal_expdSimulationPanel()),
                         this, SLOT(slot_expandSimulationPanel()));
        QObject::connect(m_TskBox_TracSimulator, SIGNAL(Signal_foldSimulationPanel()),
                         this, SLOT(slot_foldinSimulationPanel()));
        this->addWidget(m_TskBox_TracSimulator);
    }
    m_TskBox_TracSimulator->updateSimulationTarget(t_TaskObj);
    m_TskBox_TracSimulator->show();
}

void TaskDlg_TaskManage::showTskBox_SimuManage(RobotTracObject *t_TracObj)
{
    if(t_TracObj == nullptr)
        return;
    if(m_TskBox_TracSimulator == nullptr){
        m_TskBox_TracSimulator = new TaskBox_TracSimulator(t_TracObj);
        m_TskBox_TracSimulator->setMaximumHeight(220);
        m_TskBox_TracSimulator->setMaximumWidth(Maxium_Width);
        QObject::connect(m_TskBox_TracSimulator, SIGNAL(Signal_simulationOn(bool)),
                         this, SLOT(slot_disableEditing(bool)));
        QObject::connect(m_TskBox_TracSimulator, SIGNAL(Signal_expdSimulationPanel()),
                         this, SLOT(slot_expandSimulationPanel()));
        QObject::connect(m_TskBox_TracSimulator, SIGNAL(Signal_foldSimulationPanel()),
                         this, SLOT(slot_foldinSimulationPanel()));
        this->addWidget(m_TskBox_TracSimulator);
    }
    m_TskBox_TracSimulator->updateSimulationTarget(t_TracObj);
    m_TskBox_TracSimulator->show();
}

void TaskDlg_TaskManage::showTskBox_TracObject(RobotTracObject *t_TracObj)
{
    switch(t_TracObj->getTracType()){
    case Robot::TracType::FREETRAC:
        if(m_TskBox_freerunTracEditor == nullptr){
            m_TskBox_freerunTracEditor = new TaskBox_FreerunTracEditor(t_TracObj);
            QObject::connect(m_TskBox_freerunTracEditor, SIGNAL(Signal_TracEditDone(App::DocumentObject*)),
                             this, SLOT(slot_tracEditDone(App::DocumentObject*)));
            QObject::connect(m_TskBox_freerunTracEditor, SIGNAL(Signal_updateTracObject(const RobotProg_sptr)),
                             m_TskBox_TracSimulator, SLOT(slot_updateSimProgram(const RobotProg_sptr)));
            m_TskBox_freerunTracEditor->setMaximumWidth(Maxium_Width);
            this->addWidget(m_TskBox_freerunTracEditor);
        }
        else{
            m_TskBox_freerunTracEditor->setTargetTracObject(t_TracObj);
        }

        m_TskBox_freerunTracEditor->show();
        break;

    case Robot::TracType::SCANTRAC:
    case Robot::TracType::SEAMTRAC:
        if(m_TskBox_edgebasedTracEditor == nullptr){
            m_TskBox_edgebasedTracEditor = new TaskBox_EdgeBasedTracEditor(t_TracObj);
            QObject::connect(m_TskBox_edgebasedTracEditor, SIGNAL(Signal_TracObjectEditDone(App::DocumentObject*)),
                             this, SLOT(slot_tracEditDone(App::DocumentObject*)));
            QObject::connect(m_TskBox_edgebasedTracEditor, SIGNAL(Signal_updateTracObject(const RobotProg_sptr)),
                             m_TskBox_TracSimulator, SLOT(slot_updateSimProgram(const RobotProg_sptr)));

            m_TskBox_edgebasedTracEditor->setMaximumWidth(Maxium_Width);
            this->addWidget(m_TskBox_edgebasedTracEditor);
        }
        else{
            m_TskBox_edgebasedTracEditor->initEdgeTracEditor(t_TracObj);
        }
        m_TskBox_edgebasedTracEditor->show();
        break;
    }

    m_DlgCtrl->showButtons(false);
}

void TaskDlg_TaskManage::showTskBox_ActObject(ActionObject *t_ActObj)
{
    m_DlgCtrl->showButtons(false);
}


void TaskDlg_TaskManage::hideTskBoxes(bool hide)
{
    if(hide){
        if(m_TskBox_TaskManager)
            m_TskBox_TaskManager->hide();
    }
    else{
        if(m_TskBox_TaskManager)
            m_TskBox_TaskManager->show();
    }
}

// TO TASKBOX_TaskManager
bool TaskDlg_TaskManage::insertTracObject(const string opt_Name, const TracType &t_type)
{
    std::string tracName, cmmd_str;
    switch(t_type){
    case Robot::TracType::FREETRAC:
        Gui::Command::openCommand("Insert New RobotTrac");
        tracName = m_DocPtr->getUniqueObjectName("FreeTrac");
        cmmd_str = "App.activeDocument().addObject(\"Robot::RobotTracObject\",\"%s\")";
        break;
    case Robot::TracType::SCANTRAC:
        Gui::Command::openCommand("Insert New ScanTrac");
        tracName = m_DocPtr->getUniqueObjectName("ScanTrac");
        cmmd_str = "App.activeDocument().addObject(\"Robot::EdgebasedTracObject\",\"%s\")";
        break;
    case Robot::TracType::SEAMTRAC:
        Gui::Command::openCommand("Insert New SeamTrac");
        tracName = m_DocPtr->getUniqueObjectName("SeamTrac");
        cmmd_str = "App.activeDocument().addObject(\"Robot::EdgebasedTracObject\",\"%s\")";
        break;
    }
    Gui::Command::doCommand(Gui::Command::Doc, cmmd_str.c_str(),
                            tracName.c_str());
    Gui::Command::commitCommand();

    auto objPtr = m_DocPtr->getObject(tracName.c_str());
    if(objPtr != nullptr){
        // Set Trac operator
        auto tracObj = static_cast<Robot::RobotTracObject*>(objPtr);
        tracObj->setTracManager(std::string(m_taskObjPtr->getNameInDocument()));
//        tracObj->setOperator(opt_Name);
        tracObj->setTracType(t_type);
        m_taskObjPtr->insertAction(tracObj);
        showTskBox_TracObject(tracObj);
        return true;
    }

    return false;
}

const string TaskDlg_TaskManage::getDialogType() const
{
    return std::string("TaskManager");
}

#include "moc_TaskDlg_TaskManage.cpp"
