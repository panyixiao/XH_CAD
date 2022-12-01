/***************************************************************************
 *   Copyright (c) 2008 Jürgen Riegel (juergen.riegel@web.de)              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


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

#include <Mod/Robot/App/Tool/ToolObject.h>


using namespace std;

//===========================================================================
// Cmd CAD Import
//===========================================================================
DEF_STD_CMD(CmdImportCADModel)

CmdImportCADModel::CmdImportCADModel() : Command("Import_CAD_File") {
  sAppModule = "Robot";
  sGroup = QT_TR_NOOP("Robot");
  sMenuText = QT_TR_NOOP("Import CAD file");
  sToolTipText = QT_TR_NOOP("Imports a CAD file");
  sWhatsThis = QT_TR_NOOP("Import CAD file to setup planning environment");
  sStatusTip = sToolTipText;
  sPixmap = "cad_Import";
}
void CmdImportCADModel::activated(int iMsg) {
  App::Document *pDoc = getDocument();
  if (!pDoc)
    pDoc = App::GetApplication().newDocument("Work Space");

  QStringList filter;
  filter << QString::fromLatin1("STEP (*.stp *.step)");
  filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
  filter << QString::fromLatin1("IGES (*.igs *.iges)");
  filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
  filter << QString::fromLatin1("BREP (*.brp *.brep)");
  QString select;
  QString fn = Gui::FileDialog::getOpenFileName(Gui::getMainWindow(),
                                                QString(),
                                                QString(),
                                                filter.join(QLatin1String(";;")), &select);
  if (!fn.isEmpty()) {
    //        Gui::WaitCursor wc;
    App::Document *pDoc = getDocument();
    if (!pDoc)
      return; // no document
    Gui::Command::openCommand("Import CAD");
    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_CAD(\"%s\",\"%s\")",
              (const char *)fn.toUtf8(), pDoc->getName());
    if (pDoc->getObjects().size() == 1) {
      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
    }
    Gui::Command::commitCommand();
    std::list<Gui::MDIView *> views = Gui::Command::getActiveGuiDocument()->getMDIViewsOfType(
        Gui::View3DInventor::getClassTypeId());
    for (std::list<Gui::MDIView *>::iterator it = views.begin();
         it != views.end(); ++it) {
      (*it)->viewAll();
    }
  }
}

//===========================================================================
// CmdTool Import
//===========================================================================
DEF_STD_CMD(CmdImportTool)

CmdImportTool::CmdImportTool() : Command("ImportTool") {
  sAppModule = "Robot";
  sGroup = QT_TR_NOOP("Robot");
  sMenuText = QT_TR_NOOP("Import a Tool");
  sToolTipText = QT_TR_NOOP("Imports Tool");
  sWhatsThis = QT_TR_NOOP("Setup tools which can be assembled on robot");
  sStatusTip = sToolTipText;
  sPixmap = "tool";
}

void CmdImportTool::activated(int iMsg) {
  App::Document *pDoc = getDocument();
  if (!pDoc)
    pDoc = App::GetApplication().newDocument("Work Space");
  QStringList filter;
  filter << QString::fromLatin1("IGES (*.igs *.iges)");
  filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
  filter << QString::fromLatin1("STEP (*.stp *.step)");
  filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
  filter << QString::fromLatin1("BREP (*.brp *.brep)");
  filter << QString::fromLatin1("torch (*.tor)");
  filter << QString::fromLatin1("laser (*.lsr)");
  filter << QString::fromLatin1("camer (*.cam)");
  QString select;
  QString fn = Gui::FileDialog::getOpenFileName(
      Gui::getMainWindow(), QString(), QString(),
      filter.join(QLatin1String(";;")), &select);
  if (!fn.isEmpty()) {

      std::string toolName = getUniqueObjectName("Tool");
      openCommand("Insert Tool Object");
      doCommand(Doc,"App.activeDocument().addObject(\"Robot::ScannerObject\",\"%s\")",toolName.c_str());
      doCommand(Doc,"App.activeDocument().%s.CAD_File = \"%s\"",toolName.c_str(),(const char *)fn.toUtf8());
      updateActive();
      commitCommand();

//      auto result = static_cast<Robot::ToolObject*>(pDoc->getObject(toolName.c_str()));
//      if(result!=nullptr)
//          result->loadStepShape(fn.toStdString());

//    Gui::Command::openCommand("Import Tool");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
//    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_Tool(\"%s\",\"%s\")",(const char *)fn.toUtf8(), pDoc->getName());
//    updateActive();
//    if (pDoc->getObjects().size() == 1) {
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
//      Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
//    }
//    Gui::Command::commitCommand();
  }
}



// #####################################################################################################



void CreateModelOperationCommands(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdImportCADModel());
//    rcCmdMgr.addCommand(new CmdImportTool());
 }
