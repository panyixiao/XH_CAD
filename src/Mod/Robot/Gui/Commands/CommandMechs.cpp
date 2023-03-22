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

#include <Mod/Robot/App/Database/MechanicDatabase.h>
#include <Mod/Robot/App/Mechanics/MechanicGroup.h>
#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/Gui/Mechanics/MechanicSelectionPanel.h>


// #####################################################################################################
DEF_STD_CMD_A(CmdRobotInsertMechanicRobot);

CmdRobotInsertMechanicRobot::CmdRobotInsertMechanicRobot()
    :Command("Robot_InsertMechanicRobot")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("Insert a robot model");
    sToolTipText    = QT_TR_NOOP("Insert a robot model");
    sWhatsThis      = "Insert a robot model";
    sStatusTip      = sToolTipText;
    sPixmap         = "robotArm";
}


void CmdRobotInsertMechanicRobot::activated(int iMsg)
{
    App::Document *pDoc = getDocument();
    if (!pDoc)
      pDoc = App::GetApplication().newDocument("New Scene");
    Robot::MechanicDatabase m_DB;
    auto t_SelectionPtr = std::make_shared<RobotGui::MechanicSelectionPanel>(pDoc,&m_DB,Robot::MechanicType::M_Robot);
    Gui::Control().show_TaskManageDialog(t_SelectionPtr, false);
}

bool CmdRobotInsertMechanicRobot::isActive(void)
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

void CreateRobotCommandsMechOperation(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdRobotInsertMechanicRobot());
    rcCmdMgr.addCommand(new CmdRobotInsertPositioner());
    rcCmdMgr.addCommand(new CmdRobotInsertExtAxisDevice());
}
