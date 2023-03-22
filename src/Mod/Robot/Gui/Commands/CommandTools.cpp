// Insertd By Yixiao 20220615

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
# include <QMessageBox>
#endif

#include <App/Application.h>
#include <Gui/Application.h>
#include <Gui/MainWindow.h>
#include <Gui/Command.h>
#include <Gui/FileDialog.h>
#include <Gui/Selection.h>
#include <Gui/SelectionFilter.h>
#include <Gui/Document.h>
#include <Gui/Control.h>
#include <Gui/View3DInventor.h>
#include <Mod/Robot/Gui/Tool/ToolOperationPanel.h>

using namespace std;
////===========================================================================
//// Insert a WeldingTorch Tool
////===========================================================================
//DEF_STD_CMD(CmdInsertWeldTorch)

//CmdInsertWeldTorch::CmdInsertWeldTorch() :
//    Command("Command_InsertWeldTorch") {
//  sAppModule = "Robot";
//  sGroup = QT_TR_NOOP("Robot");
//  sMenuText = QT_TR_NOOP("Insert a Welding Torch");
//  sToolTipText = QT_TR_NOOP("Insert a Welding Torch");
//  sWhatsThis = QT_TR_NOOP("Insert a Welding Torch which can be assembled on robot");
//  sStatusTip = sToolTipText;
//  sPixmap = "Torch";
//}

//void CmdInsertWeldTorch::activated(int iMsg) {
//  App::Document *pDoc = getDocument();
//  if (!pDoc)
//    pDoc = App::GetApplication().newDocument("Work Space");
//  QStringList filter;
//  filter << QString::fromLatin1("STEP (*.stp *.step)");
//  filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
//  filter << QString::fromLatin1("IGES (*.igs *.iges)");
//  filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
//  filter << QString::fromLatin1("BREP (*.brp *.brep)");
//  filter << QString::fromLatin1("torch (*.torch)");
//  QString select;
//  QString fn = Gui::FileDialog::getOpenFileName(Gui::getMainWindow(),
//                                                QString(),
//                                                QString(),
//                                                filter.join(QLatin1String(";;")),
//                                                &select);
//  if (!fn.isEmpty()) {
//    Gui::Command::openCommand("Import Tool");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_Torch(\"%s\",\"%s\")",(const char *)fn.toUtf8(), pDoc->getName());
//    if (pDoc->getObjects().size() == 1) {
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
//    }
//    Gui::Command::commitCommand();
//  }
//}

//// #####################################################################################################
//===========================================================================
// Insert a WeldingTorch Tool
//===========================================================================
DEF_STD_CMD(CmdInsertWeldTorch)

CmdInsertWeldTorch::CmdInsertWeldTorch() :
    Command("Command_InsertWeldTorch") {
  sAppModule = "Robot";
  sGroup = QT_TR_NOOP("Robot");
  sMenuText = QT_TR_NOOP("Insert a Welding Torch");
  sToolTipText = QT_TR_NOOP("Insert a Welding Torch");
  sWhatsThis = QT_TR_NOOP("Insert a Welding Torch which can be assembled on robot");
  sStatusTip = sToolTipText;
  sPixmap = "Torch";
}

void CmdInsertWeldTorch::activated(int iMsg) {
  App::Document *pDoc = getDocument();
  if (!pDoc)
    pDoc = App::GetApplication().newDocument("Work Space");
  auto t_ToolOperationPanel = std::make_shared<RobotGui::ToolOperationPanel>(pDoc,Robot::ToolType::WeldTorch);
  Gui::Control().show_TaskManageDialog(t_ToolOperationPanel,false);
  //  QStringList filter;
//  filter << QString::fromLatin1("STEP (*.stp *.step)");
//  filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
//  filter << QString::fromLatin1("IGES (*.igs *.iges)");
//  filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
//  filter << QString::fromLatin1("BREP (*.brp *.brep)");
//  filter << QString::fromLatin1("torch (*.torch)");
//  QString select;
//  QString fn = Gui::FileDialog::getOpenFileName(Gui::getMainWindow(),
//                                                QString(),
//                                                QString(),
//                                                filter.join(QLatin1String(";;")),
//                                                &select);
//  if (!fn.isEmpty()) {
//    Gui::Command::openCommand("Import Tool");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_Torch(\"%s\",\"%s\")",(const char *)fn.toUtf8(), pDoc->getName());
//    if (pDoc->getObjects().size() == 1) {
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
//    }
//    Gui::Command::commitCommand();
//  }
}

// #####################################################################################################


//===========================================================================
// Insert a LaserScanner Tool
//===========================================================================
DEF_STD_CMD(CmdInsertLaserScanner)

CmdInsertLaserScanner::CmdInsertLaserScanner() :
    Command("Command_InsertLaserScanner") {
  sAppModule = "Robot";
  sGroup = QT_TR_NOOP("Robot");
  sMenuText = QT_TR_NOOP("Insert a Laser Scanner");
  sToolTipText = QT_TR_NOOP("Insert a Laser Scanner");
  sWhatsThis = QT_TR_NOOP("Insert a Laser Scannerwhich can be assembled on robot");
  sStatusTip = sToolTipText;
  sPixmap = "2DScanner";
}

void CmdInsertLaserScanner::activated(int iMsg) {
  App::Document *pDoc = getDocument();
  if (!pDoc)
    pDoc = App::GetApplication().newDocument("Work Space");
  auto t_ToolOperationPanel = std::make_shared<RobotGui::ToolOperationPanel>(pDoc,Robot::ToolType::_2DScanner);
  Gui::Control().show_TaskManageDialog(t_ToolOperationPanel,false);
//  if (!pDoc)
//    pDoc = App::GetApplication().newDocument("Work Space");
//  QStringList filter;
//  filter << QString::fromLatin1("STEP (*.stp *.step)");
//  filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
//  filter << QString::fromLatin1("IGES (*.igs *.iges)");
//  filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
//  filter << QString::fromLatin1("BREP (*.brp *.brep)");
//  filter << QString::fromLatin1("laser (*.laser)");
//  QString select;
//  QString fn = Gui::FileDialog::getOpenFileName(
//      Gui::getMainWindow(), QString(), QString(),
//      filter.join(QLatin1String(";;")), &select);
//  if (!fn.isEmpty()) {
//    Gui::Command::openCommand("Import Tool");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_LaserScanner(\"%s\",\"%s\")",(const char *)fn.toUtf8(), pDoc->getName());
//    updateActive();
//    if (pDoc->getObjects().size() == 1) {
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
//    }
//    Gui::Command::commitCommand();
//  }
}

// #####################################################################################################

//===========================================================================
// Insert a 3D Camera Tool
//===========================================================================
DEF_STD_CMD(CmdInsert3DCamera)

CmdInsert3DCamera::CmdInsert3DCamera() :
    Command("Command_Insert3DCamera") {
  sAppModule = "Robot";
  sGroup = QT_TR_NOOP("Robot");
  sMenuText = QT_TR_NOOP("Insert a 3D-Camera");
  sToolTipText = QT_TR_NOOP("Insert a 3D-Camera");
  sWhatsThis = QT_TR_NOOP("Insert a 3D-Camera which can be assembled on robot");
  sStatusTip = sToolTipText;
  sPixmap = "camera";
}

void CmdInsert3DCamera::activated(int iMsg) {
  App::Document *pDoc = getDocument();
//  if (!pDoc)
//    pDoc = App::GetApplication().newDocument("Work Space");
//  QStringList filter;
//  filter << QString::fromLatin1("STEP (*.stp *.step)");
//  filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
//  filter << QString::fromLatin1("IGES (*.igs *.iges)");
//  filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
//  filter << QString::fromLatin1("BREP (*.brp *.brep)");
//  filter << QString::fromLatin1("camer (*.camera)");
//  QString select;
//  QString fn = Gui::FileDialog::getOpenFileName(
//      Gui::getMainWindow(), QString(), QString(),
//      filter.join(QLatin1String(";;")), &select);
//  if (!fn.isEmpty()) {
//    Gui::Command::openCommand("Import Tool");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_3dCamera(\"%s\",\"%s\")",(const char *)fn.toUtf8(), pDoc->getName());
//    updateActive();
//    if (pDoc->getObjects().size() == 1) {
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
//    }
//    Gui::Command::commitCommand();
//  }
  if (!pDoc)
    pDoc = App::GetApplication().newDocument("Work Space");
  auto t_ToolOperationPanel = std::make_shared<RobotGui::ToolOperationPanel>(pDoc,Robot::ToolType::_3DCamera);
  Gui::Control().show_TaskManageDialog(t_ToolOperationPanel,false);
}

// #####################################################################################################

void CreateRobotCommandsToolOperation(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();

    rcCmdMgr.addCommand(new CmdInsertWeldTorch());
    rcCmdMgr.addCommand(new CmdInsertLaserScanner());
    rcCmdMgr.addCommand(new CmdInsert3DCamera());
 }
