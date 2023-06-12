#ifndef _PreComp_
#endif

#include <App/Document.h>
#include <App/DocumentObjectPy.h>
#include <Base/Placement.h>

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Console.h>

#include "Mod/Robot/App/Utilites/CAD_Utility.h"
#include "Mod/Robot/App/Mechanics/MechanicExtAx.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::MechanicExtAx, Robot::MechanicBase)


MechanicExtAx::MechanicExtAx()
{
    ADD_PROPERTY_TYPE(RatedLoad,(0),"属性",Prop_None,"变位机额定负载");
    ADD_PROPERTY(MountedRobotName,(""));
//    ADD_PROPERTY(AssembledToolName,(""));
    MountedRobotName.setStatus(App::Property::Status::Hidden, true);
//    AssembledToolName.setStatus(App::Property::Status::Hidden, true);
}

MechanicExtAx::~MechanicExtAx()
{
}

DocumentObjectExecReturn *MechanicExtAx::execute()
{
    return MechanicBase::execute();
}

DocumentObjectExecReturn *MechanicExtAx::recompute()
{
    return MechanicBase::recompute();
}

short MechanicExtAx::mustExecute(void) const
{
    return 0;
}

PyObject *MechanicExtAx::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}


void MechanicExtAx::onChanged(const Property* prop)
{
    if(prop == &Pose_Tip){
        updateMountedRobotPose();
    }
    Robot::MechanicBase::onChanged(prop);
}

void MechanicExtAx::onDocumentRestored()
{
    Robot::MechanicBase::onDocumentRestored();
    if(!MountedRobotName.getStrValue().empty()){
        mountMechanicRobot(MountedRobotName.getValue());
    }
//    if(!AssembledToolName.getStrValue().empty()){
//        mountMechanicRobot(AssembledToolName.getValue());
//    }
}

bool MechanicExtAx::setTipPose(const Base::Placement &n_TipPose,
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

const Base::Placement MechanicExtAx::getToolTipTranslation() const
{
//    if(m_AssembledToolPtr!=nullptr){
//        return m_AssembledToolPtr->Trans_M2T.getValue();
//    }
    return Base::Placement();
}


void MechanicExtAx::mountMechanicRobot(const char *toolName)
{
    if(toolName == nullptr)
        return;
    auto t_RobotObjPtr = static_cast<Robot::MechanicRobot*>(getDocument()->getObject(toolName));
    if(t_RobotObjPtr == nullptr)
        return;
    m_MountedRobotPtr = t_RobotObjPtr;
    updateMountedRobotPose();
    MountedRobotName.setValue(std::string(t_RobotObjPtr->getNameInDocument()));
}

void MechanicExtAx::dismountMechanicRobot()
{
    m_MountedRobotPtr = nullptr;
    MountedRobotName.setValue("");
}

void MechanicExtAx::updateMountedRobotPose()
{
//    if(m_MountedObjectPtr)
//        m_MountedObjectPtr->Pose_Mount.setValue(getCurrentTipPose());
//    if(m_AssembledToolPtr)
//        m_AssembledToolPtr->Pose_Mount.setValue(getCurrentTipPose());
    if(m_MountedRobotPtr)
        m_MountedRobotPtr->Pose_Reference.setValue(getCurrentTipPose());
}

std::vector<DocumentObject *> MechanicExtAx::getChildrenList() const
{
    std::vector<DocumentObject *> result;
    if(m_MountedRobotPtr!=nullptr)
        result.push_back(m_MountedRobotPtr);
//    if(m_AssembledToolPtr!=nullptr)
//        result.push_back(m_AssembledToolPtr);
    return result;
}


void MechanicExtAx::Save(Base::Writer &writer) const
{
    MechanicBase::Save(writer);
}

void MechanicExtAx::Restore(Base::XMLReader &reader)
{
    MechanicBase::Restore(reader);
}






