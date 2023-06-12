// Created By Yixiao 2022-05-07

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
# include <QMessageBox>
#endif

#include <App/Application.h>
#include <Gui/Application.h>
#include <Gui/Command.h>
#include <Gui/MainWindow.h>
#include <Gui/FileDialog.h>
#include <Gui/Selection.h>
#include <Gui/Document.h>

// #####################################################################################################
// Create a ComboTask
DEF_STD_CMD(CmdRobotCreateComboTask);

CmdRobotCreateComboTask::CmdRobotCreateComboTask()
    : Command("Command_CreateNewTask") {
  sAppModule = "Robot";
  sGroup = QT_TR_NOOP("Task Manage");
  sMenuText = QT_TR_NOOP("Create A New Task");
  sToolTipText = QT_TR_NOOP("Create A New Task");
  sWhatsThis = QT_TR_NOOP("Create A New Task");
  sStatusTip = QT_TR_NOOP("Create A New Task");
  sPixmap = "Robot_Edge2Trac";
}

void CmdRobotCreateComboTask::activated(int iMsg) {
  if (!getDocument()) {
    return;
  }
  std::string objName = getUniqueObjectName("Task");
  openCommand("Create A Combo Task");
  doCommand(Doc, "App.activeDocument().addObject(\"Robot::TaskObject\",\"%s\")",
            objName.c_str());
  doCommand(Gui, "Gui.activeDocument().setEdit('%s')", objName.c_str());
  commitCommand();
}

// #####################################################################################################
void CreateRobotCommandsTaskOperation(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdRobotCreateComboTask());
}
