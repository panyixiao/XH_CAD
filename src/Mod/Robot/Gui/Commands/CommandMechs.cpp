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
    sMenuText       = QT_TR_NOOP("插入机器人");
    sToolTipText    = QT_TR_NOOP("根据需求选择，插入一个机器人设备");
    sWhatsThis      = "末端可安装各类工具并进行示教编程";
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
DEF_STD_CMD_A(CmdRobotInsertMechanicPoser);

CmdRobotInsertMechanicPoser::CmdRobotInsertMechanicPoser()
    :Command("Robot_InsertPositioner")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("变位机");
    sToolTipText    = QT_TR_NOOP("根据需求选择，插入一个变位机设备");
    sWhatsThis      = "末端可安装工具或者工件，可与机器人绑定为多轴系统";
    sStatusTip      = sToolTipText;
    sPixmap         = "weldingpositioner";
}


void CmdRobotInsertMechanicPoser::activated(int iMsg)
{
    App::Document *pDoc = getDocument();
    if (!pDoc)
      pDoc = App::GetApplication().newDocument("New Scene");
    Robot::MechanicDatabase m_DB;
    auto t_SelectionPtr = std::make_shared<RobotGui::MechanicSelectionPanel>(pDoc,&m_DB,Robot::MechanicType::M_Positioner);
    Gui::Control().show_TaskManageDialog(t_SelectionPtr, false);
}

bool CmdRobotInsertMechanicPoser::isActive(void)
{
    return true;
}

// #####################################################################################################
DEF_STD_CMD_A(CmdRobotInsertMechanicExtAx);

CmdRobotInsertMechanicExtAx::CmdRobotInsertMechanicExtAx()
    :Command("Robot_InsertExtAxisDevice")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("外部轴");
    sToolTipText    = QT_TR_NOOP("根据需求选择，插入一个外部轴设备");
    sWhatsThis      = "末端可安装机器人，可与机器人绑定为多轴系统";
    sStatusTip      = sToolTipText;
    sPixmap         = "Rail";
}


void CmdRobotInsertMechanicExtAx::activated(int iMsg)
{
    App::Document *pDoc = getDocument();
    if (!pDoc)
      pDoc = App::GetApplication().newDocument("New Scene");
    Robot::MechanicDatabase m_DB;
    auto t_SelectionPtr = std::make_shared<RobotGui::MechanicSelectionPanel>(pDoc,&m_DB,Robot::MechanicType::M_ExtAxis);
    Gui::Control().show_TaskManageDialog(t_SelectionPtr, false);
}

bool CmdRobotInsertMechanicExtAx::isActive(void)
{
    return true;
}

// #####################################################################################################

void CreateRobotCommandsMechOperation(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdRobotInsertMechanicRobot());
    rcCmdMgr.addCommand(new CmdRobotInsertMechanicPoser());
    rcCmdMgr.addCommand(new CmdRobotInsertMechanicExtAx());
}
