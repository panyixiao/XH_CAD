

#include "Mod/Robot/Gui/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/BitmapFactory.h>
#include <Gui/ViewProvider.h>
#include <Gui/WaitCursor.h>
#include <Base/Console.h>
#include <Gui/Selection.h>

#include "Mod/Robot/Gui/ui_TaskBox_TracSimulator.h"
#include "TaskBox_TracSimulator.h"


using namespace RobotGui;
using namespace Gui;

//TaskBox_TracSimulator::TaskBox_TracSimulator(Robot::RobotTracObject *t_TracObj,
//                                             QWidget *parent)
//    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),tr("仿真管理"),true, parent)
//{
//    initUi(false);
//    if(!updateSimulationTarget(t_TracObj))
//        return;
//    m_SimulatorPtr = new Robot::ProgramSimulator();
//    m_SimulatorPtr->initDocPtr(m_DocPtr);
//}

TaskBox_TracSimulator::TaskBox_TracSimulator(Robot::TaskObject *t_TaskObj, QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),tr("仿真管理"),true, parent)
{
    initUi(true);
    if(!updateSimulationTarget(t_TaskObj))
        return;
    m_SimulatorPtr = new Robot::ProgramSimulator();
    m_SimulatorPtr->initDocPtr(m_DocPtr);
}

void TaskBox_TracSimulator::initUi(bool showCmdlineWidget)
{
    // we need a separate container widget to add all controls to
    proxy = new QWidget(this);
    m_ui = new Ui_TaskBox_TracSimulator();
    m_ui->setupUi(proxy);
    QMetaObject::connectSlotsByName(this);
    this->groupLayout()->addWidget(proxy);

    QObject::connect(m_ui->ButtonToStart           ,SIGNAL(clicked()),this,SLOT(slot_setToStart()));
    QObject::connect(m_ui->ButtonRunSimulation     ,SIGNAL(clicked()),this,SLOT(slot_runButtonClicked()));
    QObject::connect(m_ui->ButtonToEnd             ,SIGNAL(clicked()),this,SLOT(slot_setToEnd()));
    QObject::connect(m_ui->Button_expandPanel      ,SIGNAL(clicked()),this,SLOT(slot_changePanelSize()));

    // Timer
    timer = new QTimer( this );
    timer->setInterval(100);
    QObject::connect( timer      ,SIGNAL(timeout ()),this,SLOT(timerDone()));
//    QObject::connect( m_ui->timeSpinBox       ,SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));

    // Command List
    m_stringList = new QStringList();
    m_stringListModel = new QStringListModel(*m_stringList, NULL);
    m_ui->listView_CommandList->setModel(m_stringListModel);
    QItemSelectionModel *selectionModelPtr = m_ui->listView_CommandList->selectionModel();
    QObject::connect(selectionModelPtr, SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
                     this, SLOT(slot_changeSelectedCommand(QItemSelection)));
    m_ui->listView_CommandList->setVisible(showCmdlineWidget);
    m_ui->listView_CommandList->hide();
}

void TaskBox_TracSimulator::generateCommandExecutingMsg(const size_t t_CmdID)
{
//    auto cmdName = m_SimulatorPtr->getCurrentSimProgram()->generateCommandStr(m_SimulatorPtr->getSelectedCommandPtr(t_CmdID));
//    std::string msg = "Executing Command: " + cmdName + "...";
//    Base::Console().Message(msg.c_str());
}

//bool TaskBox_TracSimulator::updateSimulationTarget(Robot::RobotTracObject *t_TracObj)
//{
//    if(t_TracObj == nullptr)
//        return false;
//    m_DocPtr = t_TracObj->getDocument();
////    m_TracObjPtr = t_TracObj;
////    slot_updateSimProgram(m_TracObjPtr->getRobotProgramSptr());
//    return true;
//}

bool TaskBox_TracSimulator::updateSimulationTarget(Robot::TaskObject *t_TaskObj)
{
    if(t_TaskObj == nullptr)
        return false;
    m_DocPtr = t_TaskObj->getDocument();
    m_TaskObjPtr = t_TaskObj;
    slot_updateSimProgram(t_TaskObj->getTaskProgramPtr());
    return true;
}

TaskBox_TracSimulator::~TaskBox_TracSimulator()
{
    delete m_ui;
}


void TaskBox_TracSimulator::slot_changeSelectedCommand(const QItemSelection &selection)
{
//    auto t_ID = ;
//    c_CmdID = m_ui->listView_CommandList->selectionModel()->currentIndex().row();
//    auto t_cmdPtr = m_SimulatorPtr->getSelectedCommandPtr(c_CmdID);
//    m_SimulatorPtr->executeSelectedCommand(t_cmdPtr);
//    auto t_PosePtr = m_SimulatorPtr->getSelectedCommandPose(t_cmdPtr);
//    if(t_PosePtr != nullptr)
//        m_ui->label_Pos->setText(QString::fromStdString(t_PosePtr->getWP_Name()));
}

void TaskBox_TracSimulator::slot_updateSimProgram(const RobotProg_sptr t_Program)
{
    if(t_Program == nullptr)
        return;
    if(m_SimulatorPtr == nullptr){
        m_SimulatorPtr = new Robot::ProgramSimulator();
        m_SimulatorPtr->initDocPtr(m_DocPtr);
    }
    m_SimulatorPtr->setTargetSimProgram(t_Program);
    updateCommandList();
}

void TaskBox_TracSimulator::slot_updateSimProgram(const Robot::TaskObject *t_cTaskObjPtr)
{
    slot_updateSimProgram(t_cTaskObjPtr->getTaskProgramPtr());
}

void TaskBox_TracSimulator::slot_changePanelSize()
{
    if(panel_expanded){
        m_ui->listView_CommandList->hide();
        m_ui->Button_expandPanel->setText(tr("<<< Expand >>>"));
        Q_EMIT Signal_foldSimulationPanel();
        panel_expanded = false;
    }
    else{
        m_ui->listView_CommandList->show();
        m_ui->Button_expandPanel->setText(tr(">>> Shrink <<<"));
        Q_EMIT Signal_expdSimulationPanel();
        panel_expanded = true;
    }
}

void TaskBox_TracSimulator::updateCommandList()
{
    if(m_stringList == nullptr)
        return;
    m_stringList->clear();
    for (auto cmdPtr : m_SimulatorPtr->getCurrentSimProgram()->getCmmdData()) {
        auto cmdName = m_SimulatorPtr->getCurrentSimProgram()->generateCommandStr(cmdPtr);
        m_stringList->append(QString::fromStdString(cmdName));
    };
    m_stringListModel->setStringList(*m_stringList);
}

void TaskBox_TracSimulator::slot_setToStart(void)
{
    timePos = 0.0f;
    c_CmdID = 0;
    totalTime = 0.0f;
    auto t_CmdPtr = m_SimulatorPtr->getSelectedCommandPtr(c_CmdID);
    m_SimulatorPtr->executeSelectedCommand(t_CmdPtr);
    m_ui->timeSpinBox->setValue(timePos);
    flag_paused = true;
}

void TaskBox_TracSimulator::slot_setToEnd(void)
{
    timer->stop();
    c_CmdID = m_SimulatorPtr->getTotalCommandNumber() - 1;
    auto t_CmdPtr = m_SimulatorPtr->getSelectedCommandPtr(c_CmdID);
    m_SimulatorPtr->executeSelectedCommand(t_CmdPtr);
    flag_paused = true;
}

void TaskBox_TracSimulator::slot_runButtonClicked(void)
{
    if(flag_paused){
        if(c_CmdID == m_SimulatorPtr->getTotalCommandNumber())
            c_CmdID = 0;
        if(updateSimualtionTrac()){
            timePos = 0.0f;
            timer->start();
            m_ui->ButtonRunSimulation->setText(tr("||"));
            Q_EMIT Signal_simulationOn(true);
            flag_paused = false;
        }
    }
    else{
        flag_paused = true;
        timer->stop();
        m_ui->ButtonRunSimulation->setText(tr("|>"));
        Q_EMIT Signal_simulationOn(false);

    }
}

void TaskBox_TracSimulator::timerDone(void)
{
    if(timePos < duration){
        timePos += .1f;
        totalTime += .1f;
        m_ui->timeSpinBox->setValue(totalTime);
//        if(!m_SimulatorPtr->udpateGroupPoseByTime(timePos)){
//            Base::Console().Message("(FAILED!)\n");
//            Base::Console().Warning("TaskBox_TracSimulator::timerDone(): Failed to Set Pose By Time\n");
//            stopSimulation();
//        }
//        else{
//            if(m_SimulatorPtr->commandFinished(temp_CmdID)){
//                Base::Console().Message("(DONE!)\n");
//                auto t_id = c_CmdID - m_SimulatorPtr->getMoveBufferCommandNum()+temp_CmdID;
//                generateCommandExecutingMsg(t_id);
//                temp_CmdID++;
//            }
//            timer->start();
//        }
    }
    else{
        if(updateSimualtionTrac()){
            timePos = 0.0f;
            timer->start();
        }
        else{
            stopSimulation();
        }
    }
}


bool TaskBox_TracSimulator::updateSimualtionTrac()
{
    bool flag_continue = true;
    temp_CmdID = 0;
    auto t_SimProgramPtr = m_SimulatorPtr->getCurrentSimProgram();
    if(t_SimProgramPtr == nullptr)
        return false;
    auto t_cmdBuffer = t_SimProgramPtr->getCmmdData(c_CmdID);
    if(t_cmdBuffer.empty())
        return false;
    std::vector<RobotCommand_sptr> t_Commands;
    bool stop;
    for(auto i = 0; i<t_cmdBuffer.size()&&!stop ;i++){
        auto t_cmdPtr = t_cmdBuffer[i];
        switch(t_cmdPtr->getType()){
        case Robot::CommandType::ChgTool:
        case Robot::CommandType::OptTool:
            m_SimulatorPtr->executeSelectedCommand(t_cmdPtr);
            break;
        case Robot::CommandType::SetMove:{
            auto t_MovCmdPtr = static_cast<Robot::MoveCommand*>(t_cmdPtr.get());
            t_Commands.push_back(t_cmdPtr);
            if(t_MovCmdPtr->getMovePrec() == Robot::MovePrec::FINE){
                stop = true;
            }
        }
        default:
            break;
        }
        c_CmdID++;
    }
//    m_SimulatorPtr->updateMovCommandBuffer(t_Commands);
//    flag_continue &= m_SimulatorPtr->generateSimulationTrac_FromCurrentPose()>0;
//    if(flag_continue){
//        duration = m_SimulatorPtr->getDuration();
//    }
//    else{
//        std::string msg = "TaskBox_TracSimulator: Failed to Generate Trac From line: " + std::to_string(c_CmdID) + "\n";
//        Base::Console().Warning(msg.c_str());
//    }
    return flag_continue;
}

void TaskBox_TracSimulator::stopSimulation()
{
    timer->stop();
    flag_paused = true;
    timePos = 0.0;
    temp_CmdID = 0;
    m_ui->ButtonRunSimulation->setText(tr("|>"));
    Q_EMIT Signal_simulationOn(false);
}


#include "moc_TaskBox_TracSimulator.cpp"
