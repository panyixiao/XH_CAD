
#include "PreCompiled.h"
#ifndef _PreComp_
# include <Python.h>
#endif

#include <CXX/Extensions.hxx>
#include <CXX/Objects.hxx>

#include <Base/Console.h>
#include <Base/Interpreter.h>
#include <Base/PyObjectBase.h>
#include <Gui/Application.h>
#include <Gui/Language/Translator.h>
#include "Workbench.h"

#include "Mechanics/ViewProviderRobot6AxisObject.h"
#include "Mechanics/ViewProviderMechanicDevice.h"
#include "Mechanics/ViewProviderMechanicGroup.h"

#include "TaskManage/ViewProviderTaskObject.h"
#include "Utilites/DraggerUtility.h"
#include "PlanningObj/ViewProviderPlanningObj.h"

#include "Trac/ViewProviderRobotTrajectory.h"
#include "Trac/ViewProviderEdgebasedTracObject.h"

#include "Tool/ViewProviderToolObject.h"
#include "Tool/ViewProviderScannerObject.h"
#include "Tool/ViewProviderTorchObject.h"


// use a different name to CreateCommand()
void CreateRobotCommands(void);
void CreateRobotCommandsExport(void);
void CreateRobotCommandsInsertRobots(void);
void CreateRobotCommandsTrajectory(void);
void CreateModelOperationCommands(void);
void CreateRobotCommandsToolOperation(void);
void CreateRobotCommandsMechanicsOperation(void);
void CreateRobotCommandsTaskManagement(void);
void CreateRobotCommandsToolAssemble(void);

void loadRobotResource()
{
    // add resources and reloads the translators
    Q_INIT_RESOURCE(Robot);
    Gui::Translator::instance()->refresh();
}

namespace RobotGui {
class Module : public Py::ExtensionModule<Module>
{
public:
    Module() : Py::ExtensionModule<Module>("RobotGui")
    {
        initialize("This module is the RobotGui module."); // register with Python
    }

    virtual ~Module() {}

private:
};
PyObject* initModule()
{
    return Base::Interpreter().addModule(new Module);
}
} // namespace RobotGui


/* Python entry */
PyMOD_INIT_FUNC(RobotGui)
{
    if (!Gui::Application::Instance) {
        PyErr_SetString(PyExc_ImportError, "Cannot load Gui module in console application.");
//        return;
    }
    try {
        Base::Interpreter().runString("import PartGui");
        Base::Interpreter().runString("import Part");
        Base::Interpreter().runString("import Robot");
    }
    catch(const Base::Exception& e) {
        PyErr_SetString(PyExc_ImportError, e.what());
//        return;
    }
    PyObject* mod = RobotGui::initModule();
    Base::Console().Log("Loading GUI of Robot module... done\n");
//    (void)new RobotGui::Module();
//    Base::Console().Log("Loading GUI of Robot module... done\n");

    // instantiating the commands
    CreateRobotCommands();
    CreateRobotCommandsExport();
    CreateRobotCommandsTrajectory();
    CreateModelOperationCommands();

    /// New Commands
    CreateRobotCommandsMechanicsOperation();
    CreateRobotCommandsTaskManagement();
    CreateRobotCommandsToolOperation();
    CreateRobotCommandsToolAssemble();

    // addition objects
    RobotGui::Workbench                      ::init();
//    Gui::SoFCCSysDragger                     ::initClass();

    // Mechanics
    RobotGui::ViewProviderRobot6AxisObject   ::init();
    RobotGui::ViewProviderMechanicDevice     ::init();
    RobotGui::ViewProviderMechanicGroup      ::init();

    // Tool
    RobotGui::ViewProviderToolObject         ::init();
    RobotGui::ViewProviderTorchObject        ::init();
    RobotGui::ViewProviderScannerObject      ::init();

    // Obstacle
    RobotGui::ViewProviderPlanningObj        ::init();

    // Task
    RobotGui::ViewProviderTaskObject         ::init();
    // Trac
    RobotGui::ViewProviderRobotTrajectory    ::init();
    RobotGui::ViewProviderEdgebasedTracObject::init();

    loadRobotResource();
    PyMOD_Return(mod);
}
