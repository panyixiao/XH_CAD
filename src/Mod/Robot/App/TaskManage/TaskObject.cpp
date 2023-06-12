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


#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "TaskObject.h"
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/Trac/EdgebasedTracObject.h>
#include <Mod/Robot/App/Trac/RobotTracObject.h>

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::TaskObject, App::GeoFeature)


TaskObject::TaskObject()
{
    ADD_PROPERTY_TYPE(ActionList, (""),"Property", Prop_None, "The Name list of action objects in task");
    ADD_PROPERTY_TYPE(TaskOrigin, (Base::Placement()), "Property", Prop_None, "");
    ADD_PROPERTY_TYPE(refreshTracData, (false),"Property", Prop_None, "");
//    ActionList.clear();
}

TaskObject::~TaskObject()
{
}

void TaskObject::onDocumentRestored()
{
    refreshTracData.setValue(true);
}

void TaskObject::insertAction(DocumentObject *n_Action)
{
    auto actList = ActionList.getValues();
    if(n_Action == nullptr)
        return;
    bool valid = false;
//    if(n_Action->isDerivedFrom(Robot::RobotTracObject::getClassTypeId())){
//        valid = true;
//    }
//    else if(n_Action->isDerivedFrom(Robot::ActionObject::getClassTypeId())){
//        valid = true;
//    }

    if(valid){
        actList.push_back(std::string(n_Action->getNameInDocument()));
        ActionList.setValues(actList);
    }
}

void TaskObject::removeAction(const size_t action_ID)
{
    if(action_ID >= ActionList.getValues().size())
        return;
    auto actList = ActionList.getValues();
    // Remove Target from Document
    auto t_ActPtr = getDocument()->getObject(actList.at(action_ID).c_str());
    if(t_ActPtr!=nullptr)
        getDocument()->removeObject(t_ActPtr->getNameInDocument());
    // Remove Target from Action list
    actList.erase(actList.begin()+action_ID);
    ActionList.setValues(actList);
    refreshTracData.setValue(true);
}

const std::vector<DocumentObject *> TaskObject::getActionList()
{
    std::vector<DocumentObject*> result;
    for(auto name : ActionList.getValues()){
        auto actPtr = getDocument()->getObject(name.c_str());
        if(actPtr!=nullptr)
            result.push_back(actPtr);
    }
    return result;
}

DocumentObjectExecReturn *TaskObject::recompute()
{
}

short TaskObject::mustExecute(void) const
{
    return 0;
}

PyObject *TaskObject::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject);
}

void TaskObject::moveActionUp(const std::string &act_Name)
{
    auto nameList = ActionList.getValues();
    for(size_t i = 0; i<nameList.size(); i++){
        if(nameList[i] == act_Name){
            if(i-1>=0){
               auto buff = nameList[i-1];
               nameList[i-1] = act_Name;
               nameList[i] = buff;
            }
        }
    }
    ActionList.setValues(nameList);
    refreshTracData.setValue(true);
}

void TaskObject::moveActionDown(const std::string &act_Name)
{
    auto nameList = ActionList.getValues();
    for(size_t i = 0; i<nameList.size(); i++){
        if(nameList[i] == act_Name){
            if(i+1<=nameList.size()){
               auto buff = nameList[i+1];
               nameList[i+1] = act_Name;
               nameList[i] = buff;
            }
        }
    }
    ActionList.setValues(nameList);
    refreshTracData.setValue(true);
}

size_t TaskObject::getTaskTracPointNumber() const
{
    if(m_ProgramPtr!=nullptr)
        return m_ProgramPtr->getTotalWayPointCount();
    return 0;
}

bool TaskObject::exportTaskProgram(const std::string filePath, bool splitProgram)
{
    m_Export.setFlagSplit(splitProgram);
    return m_Export.generateProgramSequence(filePath,m_ProgramPtr);
}

void TaskObject::onDelete()
{
    for(auto t_ObjectName : ActionList.getValues()){
        auto t_ObjPtr = getDocument()->getObject(t_ObjectName.c_str());
        if(t_ObjPtr!=nullptr)
            getDocument()->removeObject(t_ObjectName.c_str());
    }
}

void TaskObject::onChanged(const Property* prop)
{
    if(prop == &refreshTracData){
        if(refreshTracData.getValue()){
            updateTracData();
            refreshTracData.setValue(false);
        }
    }
    App::GeoFeature::onChanged(prop);
}

bool TaskObject::updateTracData()
{
    bool first = true;
    for(auto actionName : ActionList.getValues()){
        auto objPtr = getDocument()->getObject(actionName.c_str());
//        if(objPtr && objPtr->isDerivedFrom(Robot::RobotTracObject::getClassTypeId())){
//            auto TracObjPtr = static_cast<Robot::RobotTracObject*>(objPtr);
//            if(first){
//                m_ProgramPtr = TracObjPtr->getRobotProgramSptr();
//                first = false;
//            }
//            else{
//                m_ProgramPtr->joinData(TracObjPtr->getRobotProgramSptr());
//            }
//        }
    }
    if(m_ProgramPtr != nullptr)
        return !m_ProgramPtr->getCmmdData().empty();
    return false;
}
