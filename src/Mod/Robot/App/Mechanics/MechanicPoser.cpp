#ifndef _PreComp_
#endif

#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "Mod/Robot/App/Utilites/CAD_Utility.h"
#include "Mod/Robot/App/Mechanics/MechanicPoser.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::MechanicPoser, Robot::MechanicBase)


MechanicPoser::MechanicPoser()
{
}

MechanicPoser::~MechanicPoser()
{
}

DocumentObjectExecReturn *MechanicPoser::execute()
{
    return MechanicBase::execute();
}

DocumentObjectExecReturn *MechanicPoser::recompute()
{
    return MechanicBase::recompute();
}

short MechanicPoser::mustExecute(void) const
{
    return 0;
}

PyObject *MechanicPoser::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}


void MechanicPoser::onChanged(const Property* prop)
{
    Robot::MechanicBase::onChanged(prop);
}

void MechanicPoser::onDocumentRestored()
{
    Robot::MechanicBase::onDocumentRestored();
    if(!MountedWorkingObj.getStrValue().empty()){
        mountWorkingObject(MountedWorkingObj.getValue());
    }
    if(!AssembledTool.getStrValue().empty()){
        assembleTool(AssembledTool.getValue());
    }
}

bool MechanicPoser::setTipPose(const Base::Placement &n_TipPose,
                                  CoordOrigin pose_Coord,
                                  Base::Placement coordOrigin_Pose)
{
    Base::Placement toolTrans = getToolTipTranslation();
    auto base_inv = getOriginPose().inverse();
    Base::Placement targetPose;
    switch(pose_Coord){
        case CoordOrigin::World:
        targetPose = base_inv.toMatrix() *
                     n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
            break;
        case CoordOrigin::Robot:
        targetPose = n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
            break;

        case CoordOrigin::Flan:
        targetPose = n_TipPose;
            break;

        case CoordOrigin::Object:{
        targetPose = base_inv.toMatrix() *
                     coordOrigin_Pose.toMatrix() *
                     n_TipPose.toMatrix() *
                     toolTrans.inverse().toMatrix();
        }
            break;
    }

    bool success = false;
    if(m_kinematicModel.setTo(targetPose)){
        success = true;
        Pose_Tip.setValue(targetPose);
    }

    return success;
}

const Base::Placement MechanicPoser::getToolTipTranslation() const
{
    return Base::Placement();
}


void MechanicPoser::assembleTool(const char *toolName)
{
    if(toolName == nullptr)
        return;
    auto t_ToolObjPtr = static_cast<Robot::ToolObject*>(getDocument()->getObject(toolName));
    if(t_ToolObjPtr == nullptr)
        return;
    m_AssembledToolPtr = t_ToolObjPtr;
    udpateLoadPosition();
    AssembledTool.setValue(std::string(t_ToolObjPtr->getNameInDocument()));
}

void MechanicPoser::disassembleTool()
{
    m_AssembledToolPtr = nullptr;
    AssembledTool.setValue("");
}


void MechanicPoser::mountWorkingObject(const char *obejctName)
{
    if(obejctName == nullptr)
        return;
    auto t_ObjectPtr = static_cast<Robot::PlanningObject*>(getDocument()->getObject(obejctName));
    if(t_ObjectPtr == nullptr)
        return;
    m_MountedWorkingObjPtr = t_ObjectPtr;
    udpateLoadPosition();
    MountedWorkingObj.setValue(std::string(t_ObjectPtr->getNameInDocument()));
}

void MechanicPoser::dismountWorkingObject()
{
//    m_MountedObjectPtr->Pose_Mount.setValue(Base::Placement());
    m_MountedWorkingObjPtr = nullptr;
    MountedWorkingObj.setValue("");
}

void MechanicPoser::udpateLoadPosition()
{
    if(m_MountedWorkingObjPtr)
        m_MountedWorkingObjPtr->Pose_Mount.setValue(getCurrentTipPose());
    if(m_AssembledToolPtr)
        m_AssembledToolPtr->Pose_Mount.setValue(getCurrentTipPose());
}

std::vector<DocumentObject *> MechanicPoser::getChildrenList() const
{
    std::vector<DocumentObject *> result;
    if(m_MountedWorkingObjPtr)
        result.push_back(m_MountedWorkingObjPtr);
    return result;
}


void MechanicPoser::Save(Base::Writer &writer) const
{
    MechanicBase::Save(writer);
}

void MechanicPoser::Restore(Base::XMLReader &reader)
{
    MechanicBase::Restore(reader);
}






