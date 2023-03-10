// Created By Yixiao 2022-05-07

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
# include <QMessageBox>
#endif

#include <App/Application.h>
#include <Gui/Application.h>
#include <Gui/Control.h>
#include <Gui/Command.h>
#include <Gui/MainWindow.h>
#include <Gui/FileDialog.h>
#include <Gui/Document.h>
#include <Gui/Selection.h>
#include <Gui/SelectionFilter.h>

#include <Mod/Robot/App/Mechanics/MechanicGroup.h>
#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/Gui/Mechanics/MechanicSelectionPanel.h>


// #####################################################################################################
DEF_STD_CMD_A(CmdRobotInsert6AxisRobot);

CmdRobotInsert6AxisRobot::CmdRobotInsert6AxisRobot()
    :Command("Robot_Insert6AxisRobot")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("Insert a robot model");
    sToolTipText    = QT_TR_NOOP("Insert a robot model");
    sWhatsThis      = "Insert a robot model";
    sStatusTip      = sToolTipText;
    sPixmap         = "robotArm";
}


void CmdRobotInsert6AxisRobot::activated(int iMsg)
{
    App::Document *pDoc = getDocument();
    if (!pDoc)
      pDoc = App::GetApplication().newDocument("New Scene");
    Robot::MechanicDatabase m_DB;
    auto t_SelectionPtr = std::make_shared<RobotGui::MechanicSelectionPanel>(pDoc,&m_DB,Robot::MechanicType::M_Robot);
    Gui::Control().show_TaskManageDialog(t_SelectionPtr, false);
}

bool CmdRobotInsert6AxisRobot::isActive(void)
{
    return true;
}

// #####################################################################################################
DEF_STD_CMD_A(CmdRobotInsertPositioner);

CmdRobotInsertPositioner::CmdRobotInsertPositioner()
    :Command("Robot_InsertPositioner")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("Positioner");
    sToolTipText    = QT_TR_NOOP("Insert a Positioner Device.");
    sWhatsThis      = "Insert a Positioner Device";
    sStatusTip      = sToolTipText;
    sPixmap         = "weldingpositioner";
}


void CmdRobotInsertPositioner::activated(int iMsg)
{
    App::Document *pDoc = getDocument();
    if (!pDoc)
      pDoc = App::GetApplication().newDocument("New Scene");
    Robot::MechanicDatabase m_DB;
    auto t_SelectionPtr = std::make_shared<RobotGui::MechanicSelectionPanel>(pDoc,&m_DB,Robot::MechanicType::M_Positioner);
    Gui::Control().show_TaskManageDialog(t_SelectionPtr, false);
}

bool CmdRobotInsertPositioner::isActive(void)
{
    return true;
}

// #####################################################################################################
DEF_STD_CMD_A(CmdRobotInsertExtAxisDevice);

CmdRobotInsertExtAxisDevice::CmdRobotInsertExtAxisDevice()
    :Command("Robot_InsertExtAxisDevice")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("External Axis");
    sToolTipText    = QT_TR_NOOP("Insert an External Axis Device.");
    sWhatsThis      = "Insert an External Axis Device";
    sStatusTip      = sToolTipText;
    sPixmap         = "Rail";
}


void CmdRobotInsertExtAxisDevice::activated(int iMsg)
{
    App::Document *pDoc = getDocument();
    if (!pDoc)
      pDoc = App::GetApplication().newDocument("New Scene");
    Robot::MechanicDatabase m_DB;
    auto t_SelectionPtr = std::make_shared<RobotGui::MechanicSelectionPanel>(pDoc,&m_DB,Robot::MechanicType::M_ExtAxis);
    Gui::Control().show_TaskManageDialog(t_SelectionPtr, false);
}

bool CmdRobotInsertExtAxisDevice::isActive(void)
{
    return true;
}



// #####################################################################################################
DEF_STD_CMD_A(CmdRobotSetHomePos);

CmdRobotSetHomePos::CmdRobotSetHomePos()
    :Command("Robot_SetHomePos")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("Set the home position");
    sToolTipText    = QT_TR_NOOP("Set the home position");
    sWhatsThis      = "Robot_SetHomePos";
    sStatusTip      = sToolTipText;
    sPixmap         = "Robot_SetHomePos";
}

void CmdRobotSetHomePos::activated(int iMsg)
{
   const char * SelFilter = "SELECT Robot::Robot6AxisObject COUNT 1 ";

    Gui::SelectionFilter filter(SelFilter);
    Robot::Robot6AxisObject *pcRobotObject;
    if (filter.match()) {
        pcRobotObject = static_cast<Robot::Robot6AxisObject*>(filter.Result[0][0].getObject());
    }
    else {
        QMessageBox::warning(Gui::getMainWindow(),
                             QObject::tr("Wrong selection"),
                             QObject::tr("Select one Robot to set home postion"));
        return;
    }

    pcRobotObject->setCurrentPoseAsHome();
}

bool CmdRobotSetHomePos::isActive(void)
{
    return hasActiveDocument();
}


// #####################################################################################################
DEF_STD_CMD_A(CmdRobotRestoreHomePos);

CmdRobotRestoreHomePos::CmdRobotRestoreHomePos()
    :Command("Robot_RestoreHomePos")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("Move to home");
    sToolTipText    = QT_TR_NOOP("Move to home");
    sWhatsThis      = "Robot_RestoreHomePos";
    sStatusTip      = sToolTipText;
    sPixmap         = "Robot_RestoreHomePos";
}


void CmdRobotRestoreHomePos::activated(int iMsg)
{
    const char * SelFilter = "SELECT Robot::Robot6AxisObject COUNT 1 ";

    Gui::SelectionFilter filter(SelFilter);
    Robot::Robot6AxisObject *pcRobotObject;
    if (filter.match()) {
        pcRobotObject = static_cast<Robot::Robot6AxisObject*>(filter.Result[0][0].getObject());
    }
    else {
        QMessageBox::warning(Gui::getMainWindow(), QObject::tr("Wrong selection"),
            QObject::tr("Select one Robot"));
        return;
    }
    pcRobotObject->restoreHomePose();
    pcRobotObject->updateAxisValues();
}

bool CmdRobotRestoreHomePos::isActive(void)
{
    return hasActiveDocument();
}

// #####################################################################################################

void CreateRobotCommandsMechOperation(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdRobotInsert6AxisRobot());
    rcCmdMgr.addCommand(new CmdRobotInsertPositioner());
    rcCmdMgr.addCommand(new CmdRobotInsertExtAxisDevice());
    rcCmdMgr.addCommand(new CmdRobotRestoreHomePos());
    rcCmdMgr.addCommand(new CmdRobotSetHomePos());
}
