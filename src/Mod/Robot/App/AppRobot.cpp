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


#include "PreCompiled.h"
#ifndef _PreComp_
# include <Python.h>
#endif

#include <Base/Console.h>
#include <Base/Interpreter.h>
#include <Base/PyObjectBase.h>

#include <CXX/Extensions.hxx>
#include <CXX/Objects.hxx>

#include <App/Application.h>

#include "Mechanics/MechanicBase.h"
#include "Mechanics/MechanicPoser.h"
#include "Mechanics/MechanicRobot.h"
#include "Mechanics/MechanicDevice.h"
#include "Mechanics/KinematicModel.h"
#include "Mechanics/Robot6AxisObject.h"
#include "Mechanics/MechanicGroup.h"

#include "Database/FileOperator.h"
#include "Database/ToolDatabase.h"
#include "Database/MechanicDatabase.h"

#include "Utilites/PartUtility.h"
#include "Utilites/FileIO_Utility.h"
#include "Utilites/FrameObject.h"

#include "Tool/ToolObject.h"
#include "Tool/TorchObject.h"
#include "Tool/ScannerObject.h"

#include "PlanningObj/PlanningObject.h"

#include "TaskManage/TaskObject.h"
#include "TaskManage/Action.h"
#include "TaskManage/ActionObject.h"

#include "Trac/RobotProgram.h"
#include "Trac/RobotTracObject.h"
#include "Trac/RobotWaypoint.h"
#include "Trac/EdgebasedTracObject.h"
#include "Trac/RobotCommand.h"
#include "Trac/MoveCommand.h"
#include "Trac/CoordCommand.h"
#include "Trac/ToolCommand.h"

namespace Robot {
class Module : public Py::ExtensionModule<Module>
{
private:
    MechanicDatabase m_ModelDatabase;
    ToolDatabase m_ToolDatabase;
public:
    Module() : Py::ExtensionModule<Module>("Robot")
    {
        add_varargs_method("insert_ROBOT", &Module::insertMech_Robot);
        add_varargs_method("insert_POSER", &Module::insertMech_Poser);
        add_varargs_method("insert_EXTAX", &Module::insertMech_ExtAx);
        add_varargs_method("insert_Solid", &Module::insert_Solid);

        add_varargs_method("insert_WeldTorch", &Module::insertTool_WeldTorch);
        add_varargs_method("insert_2DScanner", &Module::insertTool_2DScanner);
        add_varargs_method("insert_3DCamera", &Module::insertTool_3DCamera);
        add_varargs_method("create_WeldTorch", &Module::createTool_WeldTorch);
        add_varargs_method("create_2DScanner", &Module::createTool_2DScanner);
        add_varargs_method("create_3DCamera", &Module::createTool_3DCamera);

        initialize("This module is the Robot module."); // register with Python
    }

    virtual ~Module() {}

private:

    Py::Object setHomePos(const Py::Tuple &args){}

    Py::Object insertMech_Robot(const Py::Tuple & args){
        char *ModelInfo;
        const char*DocName;
        if (!PyArg_ParseTuple(args.ptr(), "ets","utf-8",
                              &ModelInfo,
                              &DocName))
            throw Py::Exception();
        App::Document *pcDoc = App::GetApplication().getDocument(DocName);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(DocName);
        }
        std::vector<string> result;
        DS_Utility::split(std::string(ModelInfo),'_',result);
        if(result.size()!=2){
            std::string msg = "Invalid Modelinfo segment: " + std::string(ModelInfo) + "\n";
            Base::Console().Error(msg.c_str());
            throw Py::Exception();
        }
        auto ModelBrand = result[0];
        auto ModelName = result[1];
        auto t_ItemInfo = m_ModelDatabase.getTargetRobotItemInfo(ModelBrand,ModelName);
        if(t_ItemInfo.model_Name.empty()){
            std::string msg = "Failed to get" + std::string(ModelBrand) + " " + ModelName + " from Database\n";
            Base::Console().Error(msg.c_str());
            throw Py::Exception();
        }
        auto t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcDoc->addObject("Robot::MechanicRobot",ModelName.c_str()));
        if(t_RobotPtr == nullptr){
            Base::Console().Error("Failed to insert Robot Object into Document\n");
            throw Py::Exception();
        }
        t_RobotPtr->FilePath_URDF.setValue(t_ItemInfo.model_FilePath.c_str());
        return Py::None();
    }
    Py::Object insertMech_Poser(const Py::Tuple & args){
        char *ModelInfo;
        const char*DocName;
        if (!PyArg_ParseTuple(args.ptr(), "ets","utf-8",
                              &ModelInfo,
                              &DocName))
            throw Py::Exception();
        App::Document *pcDoc = App::GetApplication().getDocument(DocName);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(DocName);
        }
        std::vector<string> result;
        DS_Utility::split(std::string(ModelInfo),'_',result);
        if(result.size()!=2){
            std::string msg = "Invalid Modelinfo segment: " + std::string(ModelInfo) + "\n";
            Base::Console().Error(msg.c_str());
            throw Py::Exception();
        }
        auto ModelBrand = result[0];
        auto ModelName = result[1];
        auto t_ItemInfo = m_ModelDatabase.getTargetPoserItemInfo(ModelBrand,ModelName);
        if(t_ItemInfo.model_Name.empty()){
            std::string msg = "Failed to get" + std::string(ModelBrand) + " " + ModelName + " from Database\n";
            Base::Console().Error(msg.c_str());
            throw Py::Exception();
        }
        auto t_RobotPtr = static_cast<Robot::MechanicDevice*>(pcDoc->addObject("Robot::MechanicDevice",ModelName.c_str()));
        if(t_RobotPtr == nullptr){
            Base::Console().Error("Failed to insert Robot Object into Document\n");
            throw Py::Exception();
        }
        t_RobotPtr->File_URDF.setValue(t_ItemInfo.model_FilePath.c_str());
        return Py::None();
    }
    Py::Object insertMech_ExtAx(const Py::Tuple & args){
        char *ModelInfo;
        const char*DocName;
        if (!PyArg_ParseTuple(args.ptr(), "ets","utf-8",
                              &ModelInfo,
                              &DocName))
            throw Py::Exception();
        App::Document *pcDoc = App::GetApplication().getDocument(DocName);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(DocName);
        }
        std::vector<string> result;
        DS_Utility::split(std::string(ModelInfo),'_',result);
        if(result.size()!=2){
            std::string msg = "Invalid Modelinfo segment: " + std::string(ModelInfo) + "\n";
            Base::Console().Error(msg.c_str());
            throw Py::Exception();
        }
        auto ModelBrand = result[0];
        auto ModelName = result[1];
        auto t_ItemInfo = m_ModelDatabase.getTargetExtAxItemInfo(ModelBrand,ModelName);
        if(t_ItemInfo.model_Name.empty()){
            std::string msg = "Failed to get" + std::string(ModelBrand) + " " + ModelName + " from Database\n";
            Base::Console().Error(msg.c_str());
            throw Py::Exception();
        }
        auto t_RobotPtr = static_cast<Robot::MechanicDevice*>(pcDoc->addObject("Robot::MechanicDevice",ModelName.c_str()));
        if(t_RobotPtr == nullptr){
            Base::Console().Error("Failed to insert Robot Object into Document\n");
            throw Py::Exception();
        }
        t_RobotPtr->File_URDF.setValue(t_ItemInfo.model_FilePath.c_str());
        return Py::None();
    }

    Py::Object insert_Solid(const Py::Tuple &args) {
      char *Name;
      const char *DocName;
      if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &Name, &DocName))
        throw Py::Exception();
      std::string EncodedName = std::string(Name);
      PyMem_Free(Name);
      Base::FileInfo file(EncodedName.c_str());
      if (file.extension().empty())
        throw Py::RuntimeError("No file extension");
      App::Document *pcDoc = App::GetApplication().getDocument(DocName);
      if (!pcDoc) {
        pcDoc = App::GetApplication().newDocument(DocName);
      }
      auto result = PartUtility::importCADFile(pcDoc, EncodedName.c_str());
      if (result) {
        auto objPtr = pcDoc->getObject(result->getNameInDocument());
        if (objPtr != nullptr &&
            objPtr->isDerivedFrom(Robot::PlanningObject::getClassTypeId())) {
//          auto planningObj = dynamic_cast<Robot::PlanningObject *>(objPtr);
//          planningObj->insertIntoCollisionWorld();
        } else {
          Base::Console().Error("Can't Find Object in Document\n");
          Base::Console().Message(
              "Suggestion: Change Object name without numbers and try again.\n");
        }
      };
      return Py::None();
    }

    Py::Object insertTool_WeldTorch(const Py::Tuple &args){
        auto result = argParser(args);
        auto str_fileName = result[0];
        auto str_docName = result[1];
        App::Document *pcDoc = App::GetApplication().getDocument(str_docName.c_str());
        Base::FileInfo file(str_fileName.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        if (file.hasExtension("tor")) {            
            auto torchPtr = static_cast<Robot::TorchObject*>(pcDoc->addObject("Robot::TorchObject", file.fileNamePure().c_str()));
            torchPtr->loadTool(str_fileName);
        }
        return Py::None();
    }
    Py::Object insertTool_2DScanner(const Py::Tuple &args){
        auto result = argParser(args);
        auto FileName = result[0];
        auto DocName = result[1];
        App::Document *pcDoc = App::GetApplication().getDocument(DocName.c_str());
        Base::FileInfo file(FileName.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        // LOAD A TOOLOBJECT from File
        if (file.hasExtension("lsr")) {
//            Robot::ScannerObject::loadTool(pcDoc, FileName);
        }
        return Py::None();
    }
    Py::Object insertTool_3DCamera(const Py::Tuple &args){
        auto result = argParser(args);
        auto FileName = result[0];
        auto DocName = result[1];
        App::Document *pcDoc = App::GetApplication().getDocument(DocName.c_str());
        Base::FileInfo file(FileName.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        if (file.hasExtension("cam")) {
//            Robot::ToolObject::loadTool(FileName);
        }
        return Py::None();
    }

    Py::Object createTool_WeldTorch(const Py::Tuple & args){
        auto result = argParser(args);
        auto modelPath = result[0];
        auto DocName = result[1];
        App::Document *pcDoc = App::GetApplication().getDocument(DocName.c_str());
        Base::FileInfo file(modelPath.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        auto torchPtr = static_cast<Robot::TorchObject*>(pcDoc->addObject("Robot::TorchObject", file.fileNamePure().c_str()));
        if(torchPtr == nullptr){
            throw Py::RuntimeError("Failed to Insert Object");
        }
        if(file.hasExtension("stp") || file.hasExtension("step")){
            torchPtr->loadShape(modelPath, Robot::ShapeType::STP_Shape);
        }
        else if(file.hasExtension("igs") || file.hasExtension("iges")){
            torchPtr->loadShape(modelPath, Robot::ShapeType::IGS_Shape);
        }
        else if(file.hasExtension("obj") || file.hasExtension("stl")){
            torchPtr->loadShape(modelPath, Robot::ShapeType::MSH_Shape);
        }
        torchPtr->setEdit.setValue(true);
        torchPtr->FilePath_Solid.setValue(modelPath);
        torchPtr->FilePath_Param.setValue(m_ToolDatabase.getResFilePath());
        torchPtr->CurToolType.setValue((int)ToolType::WeldTorch);
        return Py::None();
    }
    Py::Object createTool_2DScanner(const Py::Tuple & args){

    }
    Py::Object createTool_3DCamera(const Py::Tuple & args){

    }

//    Py::Object insert_Tool(const Py::Tuple &args) {
//      auto result = argParser(args);
//      auto FileName = result[0];
//      auto DocName = result[1];
//      App::Document *pcDoc = App::GetApplication().getDocument(DocName.c_str());
//      Base::FileInfo file(result[0].c_str());
//      if (file.extension().empty())
//        throw Py::RuntimeError("No file extension");
//      // LOAD A TOOLOBJECT from File
//      if (file.hasExtension("tor")||
//          file.hasExtension("lsr")||
//          file.hasExtension("cam")) {
//          Robot::ToolObject::loadTool(pcDoc, FileName);
//      }
//      // GENERATE A NEW Tool
//      else {
//          Base::FileInfo fi(FileName);
//          if (!fi.exists()) {
//            std::stringstream str;
//            str << "File '" << FileName << "' does not exist!";
//            throw Base::RuntimeError(str.str().c_str());
//          }
//          auto toolObjPtr = static_cast<Robot::ToolObject*>(pcDoc->addObject("Robot::ToolObject", fi.fileNamePure().c_str()));
//      }
//      return Py::None();
//    }

    std::vector<string> argParser(const Py::Tuple &args){
        std::vector<string> result;
        char *arg_part1;
        const char *arg_part2;
        if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &arg_part1, &arg_part2))
          throw Py::Exception();
        // Ensure there is a doc
        App::Document *pcDoc = App::GetApplication().getDocument(arg_part2);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(arg_part2);
        }
        // Deal with file
        result.push_back(std::string(arg_part1));
        PyMem_Free(arg_part1);
        result.push_back(std::string(arg_part2));
        return result;
    }
};

PyObject* initModule()
{
    return Base::Interpreter().addModule(new Module);
}

} // namespace Robot


/* Python entry */
PyMOD_INIT_FUNC(Robot)
{
    // load dependent module
    try {
        Base::Interpreter().runString("import Part");
    }
    catch(const Base::Exception& e) {
        PyErr_SetString(PyExc_ImportError, e.what());
//        return;
    }

    PyObject* robotModule = Robot::initModule();
//    PyObject* robotModule = (new Robot::Module())->module().ptr();
    Base::Console().Log("Loading Robot module... done\n");

    // NOTE: To finish the initialization of our own type objects we must
    // call PyType_Ready, otherwise we run into a segmentation fault, later on.
    // This function is responsible for adding inherited slots from a type's base class.

    // Mechanics
    Robot::KinematicModel          ::init();
    Robot::MechanicBase            ::init();
    Robot::MechanicPoser           ::init();
    Robot::MechanicRobot           ::init();
    Robot::Robot6AxisObject        ::init();
    Robot::MechanicDevice          ::init();
    Robot::MechanicGroup           ::init();

    // Frame
    Robot::FrameObject             ::init();
    // Object
    Robot::PlanningObject          ::init();
    // Tool
    Robot::ToolObject              ::init();
    Robot::ScannerObject           ::init();
    Robot::TorchObject             ::init();
    // Task
    Robot::Action                  ::init();
    Robot::ActionObject            ::init();
    Robot::TaskObject              ::init();
    // Trac
    Robot::RobotProgram            ::init();
    Robot::RobotTracObject         ::init();
    Robot::RobotWaypoint           ::init();
    Robot::EdgebasedTracObject     ::init();
    // Command
    Robot::RobotCommand            ::init();
    Robot::MoveCommand             ::init();
    Robot::CoordCommand            ::init();
    Robot::ToolCommand             ::init();

    PyMOD_Return(robotModule);
}
