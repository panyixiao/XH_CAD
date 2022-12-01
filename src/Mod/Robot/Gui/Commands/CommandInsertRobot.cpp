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
#include <Gui/Command.h>
#include <Gui/MainWindow.h>
#include <Gui/FileDialog.h>
#include <Gui/Selection.h>
#include <Gui/Document.h>

#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>

using namespace std;

DEF_STD_CMD_A(CmdAssembleTorchModelToRobot);

CmdAssembleTorchModelToRobot::CmdAssembleTorchModelToRobot()
    :Command("Robot_AssembleTorchTool")
{
    sAppModule      = "Robot";
    sGroup          = QT_TR_NOOP("Robot");
    sMenuText       = QT_TR_NOOP("Add tool");
    sToolTipText    = QT_TR_NOOP("Add a torch shape to the robot");
    sWhatsThis      = "Robot_AssembleTorchTool";
    sStatusTip      = sToolTipText;
    sPixmap         = "ToolAndSetup";
}


void CmdAssembleTorchModelToRobot::activated(int iMsg)
{
    std::vector<App::DocumentObject*> robots = getSelection().getObjectsOfType(Robot::Robot6AxisObject::getClassTypeId());
    std::vector<App::DocumentObject*> shapes = getSelection().getObjectsOfType(Base::Type::fromName("Part::Feature"));
//    std::vector<App::DocumentObject*> meshes = getSelection().getObjectsOfType(Base::Type::fromName("Mesh::Feature"));

    if (robots.size() != 1 || shapes.empty()) {
        QMessageBox::warning(Gui::getMainWindow(), QObject::tr("Wrong selection"),
            QObject::tr("Select one robot and one Tool(part)."));
        return;
    }

    std::string RoboName = robots.front()->getNameInDocument();
    std::string ShapeName;
    if(shapes.size() == 1){
        ShapeName = shapes.front()->getNameInDocument();
        openCommand("Add tool to robot");
        doCommand(Doc,"App.activeDocument().%s.TorchShape = App.activeDocument().%s",RoboName.c_str(),ShapeName.c_str());
        updateActive();
        commitCommand();
    }
}

bool CmdAssembleTorchModelToRobot::isActive(void)
{
    return hasActiveDocument();
}

// #####################################################################################################
void CreateRobotCommandsToolAssemble(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdAssembleTorchModelToRobot());
}
