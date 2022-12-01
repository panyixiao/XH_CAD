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

#include "Mechanics/MechanicDatabase.h"
#include "Mechanics/MechanicDevice.h"
#include "Mechanics/KinematicModel.h"
#include "Mechanics/Robot6AxisObject.h"
#include "Mechanics/MechanicGroup.h"

#include "Utilites/PartUtility.h"
#include "Utilites/FileIO_Utility.h"
#include "Utilites/FrameObject.h"

#include "Tool/ToolObject.h"
#include "Tool/ScannerObject.h"
#include "Tool/TorchObject.h"

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
    MechanicDatabase m_ModelDatabase;
public:
    Module() : Py::ExtensionModule<Module>("Robot")
    {
        add_varargs_method("insert_ROBOT", &Module::insert_Robot);
        add_varargs_method("insert_POSER", &Module::insert_Poser);
        add_varargs_method("insert_EXTAX", &Module::insert_ExtAx);
        add_varargs_method("insert_CAD", &Module::insert_CAD);
        add_varargs_method("insert_Tool", &Module::insert_Tool);
        add_varargs_method("insert_Torch", &Module::insert_Torch);
        add_varargs_method("insert_LaserScanner", &Module::insert_LaserScanner);
        add_varargs_method("insert_3dCamera", &Module::insert_3dCamera);
        initialize("This module is the Robot module."); // register with Python
    }

    virtual ~Module() {}

private:

    Py::Object setHomePos(const Py::Tuple &args){}

    Py::Object insert_Robot(const Py::Tuple & args){
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
        auto t_RobotPtr = static_cast<Robot::Robot6AxisObject*>(pcDoc->addObject("Robot::Robot6AxisObject",ModelName.c_str()));
        if(t_RobotPtr == nullptr){
            Base::Console().Error("Failed to insert Robot Object into Document\n");
            throw Py::Exception();
        }
        t_RobotPtr->File_URDF.setValue(t_ItemInfo.model_FilePath.c_str());
//        PyMem_Free(Name);
//        Base::FileInfo file(EncodedName.c_str());
//        if (file.extension().empty())
//          throw Py::RuntimeError("No file extension");

//        auto result = PartUtility::importCADFile(pcDoc, EncodedName.c_str());
//        if (result) {
//          auto objPtr = pcDoc->getObject(result->getNameInDocument());
//          if (objPtr != nullptr &&
//              objPtr->isDerivedFrom(Robot::PlanningObject::getClassTypeId())) {
//            auto planningObj = dynamic_cast<Robot::PlanningObject *>(objPtr);
//  //          planningObj->insertIntoCollisionWorld();
//          } else {
//            Base::Console().Error("Can't Find Object in Document\n");
//            Base::Console().Message("Suggestion: Change Object name without numbers and try again.\n");
//          }
//        };
        return Py::None();
    }

    Py::Object insert_Poser(const Py::Tuple & args){

    }

    Py::Object insert_ExtAx(const Py::Tuple & args){

    }

    Py::Object insert_CAD(const Py::Tuple &args) {
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
          auto planningObj = dynamic_cast<Robot::PlanningObject *>(objPtr);
//          planningObj->insertIntoCollisionWorld();
        } else {
          Base::Console().Error("Can't Find Object in Document\n");
          Base::Console().Message(
              "Suggestion: Change Object name without numbers and try again.\n");
        }
      };
      return Py::None();
    }

    Py::Object insert_Torch(const Py::Tuple &args){
        char *filePath;
        const char *DocName;
        if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &filePath, &DocName))
          throw Py::Exception();
        // Ensure there is a doc
        App::Document *pcDoc = App::GetApplication().getDocument(DocName);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(DocName);
        }
        // Deal with file
        std::string EncodedName = std::string(filePath);
        PyMem_Free(filePath);
        Base::FileInfo file(EncodedName.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        // LOAD A TOOLOBJECT from File
        if (file.hasExtension("tor")) {
            Robot::ToolObject::loadTool(pcDoc, EncodedName);
        }else{
            auto torchPtr = static_cast<Robot::TorchObject*>(pcDoc->addObject("Robot::TorchObject", file.fileNamePure().c_str()));
            if(torchPtr == nullptr){
                throw Py::RuntimeError("Failed to Insert Object");
            }
            if(file.hasExtension("stp") || file.hasExtension("step")){
                torchPtr->loadShape(filePath, Robot::ShapeType::STP_Shape);
            }
            else if(file.hasExtension("igs") || file.hasExtension("iges")){
                torchPtr->loadShape(filePath, Robot::ShapeType::IGS_Shape);
            }
            else if(file.hasExtension("obj") || file.hasExtension("stl")){
                torchPtr->loadShape(filePath, Robot::ShapeType::MSH_Shape);
            }
//            torchPtr->setEdit.setValue(true);
        }
        return Py::None();
    }

    Py::Object insert_LaserScanner(const Py::Tuple &args){
        char *filePath;
        const char *DocName;
        if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &filePath, &DocName))
          throw Py::Exception();
        // Ensure there is a doc
        App::Document *pcDoc = App::GetApplication().getDocument(DocName);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(DocName);
        }
        // Deal with file
        std::string EncodedName = std::string(filePath);
        PyMem_Free(filePath);
        Base::FileInfo file(EncodedName.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        // LOAD A TOOLOBJECT from File
        if (file.hasExtension("lsr")) {
            Robot::ToolObject::loadTool(pcDoc, EncodedName);
        }
        else{
            auto scannerPtr = static_cast<Robot::ScannerObject*>(pcDoc->addObject("Robot::ScannerObject", file.fileNamePure().c_str()));
            if(scannerPtr == nullptr){
                throw Py::RuntimeError("Failed to Insert Object");
            }
            if(file.hasExtension("stp") || file.hasExtension("step")){
                scannerPtr->loadShape(filePath, Robot::ShapeType::STP_Shape);
            }
            else if(file.hasExtension("igs") || file.hasExtension("iges")){
                scannerPtr->loadShape(filePath, Robot::ShapeType::IGS_Shape);
            }
            else if(file.hasExtension("obj") || file.hasExtension("stl")){
                scannerPtr->loadShape(filePath, Robot::ShapeType::MSH_Shape);
            }
        }
        return Py::None();
    }

    Py::Object insert_3dCamera(const Py::Tuple &args){
        char *filePath;
        const char *DocName;
        if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &filePath, &DocName))
          throw Py::Exception();
        // Ensure there is a doc
        App::Document *pcDoc = App::GetApplication().getDocument(DocName);
        if (!pcDoc) {
          pcDoc = App::GetApplication().newDocument(DocName);
        }
        // Deal with file
        std::string EncodedName = std::string(filePath);
        PyMem_Free(filePath);
        Base::FileInfo file(EncodedName.c_str());
        if (file.extension().empty())
          throw Py::RuntimeError("No file extension");
        // LOAD A TOOLOBJECT from File
        if (file.hasExtension("cam")) {
            Robot::ToolObject::loadTool(pcDoc, EncodedName);
        }else if(file.hasExtension("stp")){

        }else if(file.hasExtension("iges")){

        }
        return Py::None();
    }

    Py::Object insert_Tool(const Py::Tuple &args) {
      char *filePath;
      const char *DocName;
      if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &filePath, &DocName))
        throw Py::Exception();
      // Ensure there is a doc
      App::Document *pcDoc = App::GetApplication().getDocument(DocName);
      if (!pcDoc) {
        pcDoc = App::GetApplication().newDocument(DocName);
      }
      // Deal with file
      std::string EncodedName = std::string(filePath);
      PyMem_Free(filePath);
      Base::FileInfo file(EncodedName.c_str());
      if (file.extension().empty())
        throw Py::RuntimeError("No file extension");
      // LOAD A TOOLOBJECT from File
      if (file.hasExtension("tor")||
          file.hasExtension("lsr")||
          file.hasExtension("cam")) {
          Robot::ToolObject::loadTool(pcDoc, EncodedName);
      }
      // GENERATE A NEW Tool
      else {
          Base::FileInfo fi(filePath);
          if (!fi.exists()) {
            std::stringstream str;
            str << "File '" << filePath << "' does not exist!";
            throw Base::RuntimeError(str.str().c_str());
          }
          auto toolObjPtr = static_cast<Robot::ToolObject*>(pcDoc->addObject("Robot::ToolObject", fi.fileNamePure().c_str()));
//          if(toolObjPtr!=nullptr){
//              toolObjPtr->loadStepShape(filePath);
//          }
      }
      return Py::None();
    }

    Py::Object assembleTool(const Py::Tuple &args) {
      char *Name;
      const char *DocName;
      if (!PyArg_ParseTuple(args.ptr(), "ets", "utf-8", &Name, &DocName))
        throw Py::Exception();
      App::Document *pcDoc = App::GetApplication().getDocument(DocName);
      auto robots =
          pcDoc->getObjectsOfType(Robot::Robot6AxisObject::getClassTypeId());
      auto tools =
          pcDoc->getObjectsOfType(Robot::ToolObject::getClassTypeId());
      if (robots.empty() || tools.empty())
        throw Py::RuntimeError(
            "No Robot Object or Tool Object available in document");
      for (auto T_objPtr : tools) {
        auto toolPtr = dynamic_cast<Robot::Robot6AxisObject *>(T_objPtr);
        for (auto t_robot : robots) {
          auto rbtPtr = dynamic_cast<Robot::Robot6AxisObject *>(t_robot);

        }
      }
      return Py::None();
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
