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

#include "RobotTracObject.h"
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Robot/App/Mechanics/MechanicGroup.h>

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::RobotTracObject, App::GeoFeature)


RobotTracObject::RobotTracObject()
{
    ADD_PROPERTY_TYPE( TracOrigin,      (Base::Placement())  , "Trajectory",Prop_None,"Actuall base frame of the trajectory");
    ADD_PROPERTY_TYPE( WaypointNumber, (0),"Tracjectory", Prop_None, "Actual Waypoint Numbers this Trac Contained");
    ADD_PROPERTY_TYPE( TracManagerName, (""), "Trajectory", Prop_None,"The Name of the Task this TracObject belongs to");
    ADD_PROPERTY_TYPE( ExecutorName, (""), "Trajectory", Prop_None,"The Name of the Robot to Operate this Tracjectory");
    ADD_PROPERTY(TracTypeID,(0));
//    m_ProgramPtr = ;
}

RobotTracObject::~RobotTracObject()
{

}

short RobotTracObject::mustExecute(void) const
{
    return 0;
}


PyObject *RobotTracObject::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject);
}

void RobotTracObject::insertCMD_MOVE(const std::string executorName,
                                     const RobotWaypoint &t_Pnt,
                                     const Robot::MoveType t_Type,
                                     const Robot::MovePrec t_Prec,
                                     const float Vel,
                                     const float _BL,
                                     const float _VBL)
{
    auto hashID = m_ProgramPtr->addNewPose(t_Pnt);
    m_ProgramPtr->insertCMD_NewMOVE(executorName,hashID,t_Type,t_Prec,Vel,_BL,_VBL);
    // Drive Gui Update
    WaypointNumber.setValue(m_ProgramPtr->getWaypointData().size());
}

void RobotTracObject::insertCMD_SwitchTool(const std::string executorName,
                                           const Robot::ToolType& t_Type)
{ 
    uint coordID = 0;
    auto t_Robot = static_cast<Robot::Robot6AxisObject*>(getDocument()->getObject(executorName.c_str()));
    if(t_Robot!=nullptr){
        switch(t_Type){
        case Robot::ToolType::WeldTorch:
            coordID = t_Robot->TorchIndex.getValue();
            break;
        case Robot::ToolType::Scanner:
            coordID = t_Robot->ScannerIndex.getValue();
            break;
        default:
            break;
        }
    }
    m_ProgramPtr->insertCMD_SwitchTool(executorName,t_Type,coordID);
}

void RobotTracObject::insertCMD_SetToolActivate(const string executorName, const ToolType &t_Type)
{
    m_ProgramPtr->insertCMD_SetToolOn(executorName,t_Type);
}

void RobotTracObject::insertCMD_SetToolDeActivate(const string executorName, const ToolType &t_Type)
{
    m_ProgramPtr->insertCMD_SetToolOff(executorName,t_Type);
}

void RobotTracObject::insertCMD_ChangeCord(const std::string executorName, const CordType &t_Type, const uint coordID)
{
    m_ProgramPtr->insertCMD_ChgCord(executorName, t_Type, coordID);
}

void RobotTracObject::setWaypointPose(const int pntID, const uint rbtID, const Base::Placement &t_pose)
{
    m_ProgramPtr->setWPntCartPose(pntID,rbtID, t_pose);
}

void RobotTracObject::setWaypointAdjust(const int pntID, const uint rbtID, const Base::Placement &t_adjust)
{
    m_ProgramPtr->setWPntPoseAdjust(pntID,rbtID, t_adjust);
}

void RobotTracObject::setAdjustPoseToRest(const int s_PntID, const uint rbtID, const Base::Placement &t_adjust)
{
    for(int i = s_PntID; i<m_ProgramPtr->getTotalWayPointCount(); i++){
        m_ProgramPtr->setWPntPoseAdjust(i,rbtID,t_adjust);
    }
}

//const Base::Placement RobotTracObject::getWaypointAbsPose_byID(const size_t poseID)
//{
//    return m_ProgramPtr->getWaypoint_byPosition(poseID)->getWPCartPose_GP1();
//}

//const Base::Placement RobotTracObject::getWaypointOriPose_byID(const size_t poseID)
//{
//    return m_ProgramPtr->getWaypoint_byPosition(poseID)->getWP_OriCartPose();
//}




bool RobotTracObject::removeTargetCommand(const int cmdID)
{
    if(m_ProgramPtr->deleteCMD_byPosition(cmdID)){
        WaypointNumber.setValue(m_ProgramPtr->getTotalWayPointCount());
        return true;
    }
    return false;
}

void RobotTracObject::Save(Base::Writer &writer) const
{
    App::GeoFeature::Save(writer);
    m_ProgramPtr->Save(writer);
}

void RobotTracObject::Restore(Base::XMLReader &reader)
{
    m_ProgramPtr = std::make_shared<Robot::RobotProgram>();
    m_ProgramPtr->Restore(reader);
    App::GeoFeature::Restore(reader);
}

void RobotTracObject::onDocumentRestored(){
    m_TYPE = (TracType)TracTypeID.getValue();
}

void RobotTracObject::setTracManager(const string &t_Name)
{
    TracManagerName.setValue(t_Name.c_str());
    m_ProgramPtr->setTaskName(t_Name);
}

DocumentObject *RobotTracObject::getTracManager() const
{
    return this->getDocument()->getObject(TracManagerName.getValue());
}

void RobotTracObject::setTracType(const TracType &t_Type)
{
    m_TYPE = t_Type;
    TracTypeID.setValue((int)m_TYPE);
}

const TracType &RobotTracObject::getTracType() const
{
    return m_TYPE;
}

void RobotTracObject::setOperator(const std::string &t_Name)
{
    auto executorPtr = this->getDocument()->getObject(t_Name.c_str());
    if(executorPtr->isDerivedFrom(Robot::Robot6AxisObject::getClassTypeId()) ||
       executorPtr->isDerivedFrom(Robot::MechanicGroup::getClassTypeId())){
        ExecutorName.setValue(t_Name);
    }
}

bool RobotTracObject::isEmpty()
{
    return m_ProgramPtr->getCmmdData().empty();
}

void RobotTracObject::onChanged(const Property* prop)
{
    if(prop == &TracOrigin){
        m_ProgramPtr->setTracDataOrigin(TracOrigin.getValue());
//        m_Program.updateSimTracData();
    }
    App::GeoFeature::onChanged(prop);
}

void RobotTracObject::resetData()
{
    m_ProgramPtr->clearData();
    WaypointNumber.setValue(0);
}
