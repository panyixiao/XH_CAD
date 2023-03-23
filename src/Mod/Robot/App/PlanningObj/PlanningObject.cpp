#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "PlanningObject.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Command.h>
#include "Mod/Robot/App/Utilites/CAD_Utility.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include <Mod/Robot/App/Tool/ToolObject.h>
#include <Mod/Robot/App/Mechanics/MechanicPoser.h>

using namespace Part;
using namespace App;
using namespace Robot;

PROPERTY_SOURCE(Robot::PlanningObject, Part::Feature)

PlanningObject::PlanningObject() {
  ADD_PROPERTY_TYPE(AttachedTo, (std::string("")), "Object Info", Prop_None, "Object Name");
  ADD_PROPERTY_TYPE(FilePath_Solid, (std::string("")), "File Info", Prop_None, "File Path to CAD");
  AttachedTo.setStatus(App::Property::Status::ReadOnly, true);
  FilePath_Solid.setStatus(App::Property::Status::ReadOnly, true);

  ADD_PROPERTY(tCurvFeature, (0));
  tCurvFeature.setStatus(App::Property::Hidden, true);
  ADD_PROPERTY(tFaceFeature, (0));
  tFaceFeature.setStatus(App::Property::Hidden, true);

  ADD_PROPERTY(Pose_Mount,(Base::Placement()));
  ADD_PROPERTY(Trans_O2M, (Base::Placement()));
  ADD_PROPERTY(Trans_O2F, (Base::Placement()));
  ADD_PROPERTY(isEditing, (false));

  ADD_PROPERTY_TYPE(FrameOn, (false),"Object Info", Prop_None, "Frame Visiable Switch");

//  Trans_O2F.setStatus(App::Property::Status::Hidden, true);
  Placement.setStatus(App::Property::Hidden,true);

}

void PlanningObject::insertIntoCollisionWorld() {

}

const Base::Placement PlanningObject::getCurrentMountPose()
{
    return Placement.getValue() * Trans_O2M.getValue();
}

const Base::Placement PlanningObject::getCurrentFramePose()
{
    if(Trans_O2F.getValue().isIdentity())
        Trans_O2F.setValue(Trans_O2M.getValue());
    return Placement.getValue() * Trans_O2F.getValue();
}

void PlanningObject::onChanged(const Property *prop)
{
    Part::Feature::onChanged(prop);
}

void PlanningObject::setMountPose_toFeatureCenter() {
    Base::Placement t_Center;
    auto ref_Edge = CAD_Utility::getEdgesFromDataSource(tCurvFeature);
    if (ref_Edge.size()) {
        t_Center = CAD_Utility::getCurveCenterPnt(ref_Edge.front());
    }
    else{
        t_Center = CAD_Utility::calculateLinkedFaceCenter(tFaceFeature);
    }
    Trans_O2M.setValue(Placement.getValue().inverse()*t_Center);
}

void PlanningObject::setFramePose_toFeatureCenter()
{
    Base::Placement t_Center;
    auto ref_Edge = CAD_Utility::getEdgesFromDataSource(tCurvFeature);
    if (ref_Edge.size()) {
        t_Center = CAD_Utility::getCurveCenterPnt(ref_Edge.front());
    }
    else{
        t_Center = CAD_Utility::calculateLinkedFaceCenter(tFaceFeature);
    }
    Trans_O2F.setValue(Placement.getValue().inverse()*t_Center);
}

bool PlanningObject::changeMountState(const char *targetName, bool attachTo)
{
    bool success = false;
    auto t_TargetDevicePtr = this->getDocument()->getObject(targetName);
    if(t_TargetDevicePtr->isDerivedFrom(Robot::MechanicPoser::getClassTypeId())){
        auto t_PoserPtr = static_cast<Robot::MechanicPoser*>(t_TargetDevicePtr);
        if(attachTo){
            t_PoserPtr->mountWorkingObject(this->getNameInDocument());
            Pose_Mount.setValue(t_PoserPtr->getCurrentTipPose());
            AttachedTo.setValue(t_PoserPtr->getNameInDocument());
        }else{
            t_PoserPtr->dismountWorkingObject();
            Pose_Mount.setValue(Base::Placement());
            AttachedTo.setValue("");
        }
        success = true;
    }
    else if(t_TargetDevicePtr->isDerivedFrom(Robot::ToolObject::getClassTypeId())){
        auto t_ToolPtr = static_cast<Robot::ToolObject*>(t_TargetDevicePtr);
        if(attachTo){

            Pose_Mount.setValue(t_ToolPtr->getPose_ABSToolTip());
            AttachedTo.setValue(t_ToolPtr->getNameInDocument());
        }else{

            Pose_Mount.setValue(Base::Placement());
            AttachedTo.setValue("");
        }
        success = true;
    }
    return success;
}


void PlanningObject::Save(Base::Writer &writer) const {
  Part::Feature::Save(writer);
}

void PlanningObject::Restore(Base::XMLReader &reader) {
  Part::Feature::Restore(reader);
}

void PlanningObject::onDocumentRestored() {

}



