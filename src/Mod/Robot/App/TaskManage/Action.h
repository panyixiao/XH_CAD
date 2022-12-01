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


#ifndef ROBOT_Action_H
#define ROBOT_Action_H

#include <Base/Persistence.h>
#include <Base/Placement.h>

enum class ActionTYPE{
    Basic,
    ROBOT_CMD,
    EQUIP_CMD
};


namespace Robot
{

class RobotExport Action : public Base::Persistence
{
//    PROPERTY_HEADER(Robot::Action);

    TYPESYSTEM_HEADER();

public:
    /// Constructor
    Action(void);
    virtual ~Action();
    void setTaskName(const std::string& t_Name){
        _taskName = t_Name;
    }
    void setOperatorName(const std::string& t_Name){
        _operatorName = t_Name;
    }

//    void setActionTYPE(const ActionTYPE& t_Type){
//        _type = t_Type;
//    }

    void setActionDuration(const double& t_time){
        _duration = t_time;
    }

    const std::string& getTaskName() const {
        return _taskName;
    }
    const std::string& getOperatorName() const {
        return _operatorName;
    }
    const double getActionDuration() const {
        return _duration;
    }
//    virtual const ActionTYPE getActionType() const{
//        return _type;
//    }

    // From base class
    virtual unsigned int getMemSize (void) const{ return 0;}
    virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

//    const ActionTYPE& getActionType() const{ return _type;}

protected:
    double _duration = -1;
    std::string _taskName;
    std::string _operatorName;
//    ActionTYPE _type = ActionTYPE::Basic;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H
