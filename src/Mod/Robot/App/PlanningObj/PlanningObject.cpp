#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "PlanningObject.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Command.h>
#include "Mod/Robot/App/Utilites/CAD_Utility.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"

using namespace Part;
using namespace App;
using namespace Robot;

PROPERTY_SOURCE(Robot::PlanningObject, Part::Feature)

PlanningObject::PlanningObject() {
  ADD_PROPERTY_TYPE(ObjectName, (std::string("")), "Object Info", Prop_None, "Object Name");
  ADD_PROPERTY_TYPE(FilePath, (std::string("")), "File Info", Prop_None, "File Path to CAD");
  ObjectName.setStatus(App::Property::Status::ReadOnly, true);
  FilePath.setStatus(App::Property::Status::ReadOnly, true);

  ADD_PROPERTY(tCurvFeature, (0));
  tCurvFeature.setStatus(App::Property::Hidden, true);
  ADD_PROPERTY(tFaceFeature, (0));
  tFaceFeature.setStatus(App::Property::Hidden, true);

  ADD_PROPERTY(Translation_O2M, (Base::Placement()));
  ADD_PROPERTY(Pose_Mount,(Base::Placement()));

  Placement.setStatus(App::Property::Hidden,true);
}

void PlanningObject::insertIntoCollisionWorld() {

}

void PlanningObject::updateTranslation_Origin2Mount(const Base::Placement &t_Translation)
{
    Translation_O2M.setValue(t_Translation);
}

const Base::Placement PlanningObject::getCurrentMountPose()
{
    return Placement.getValue() * Translation_O2M.getValue();
}


void PlanningObject::updateObjectMountPose(const Base::Placement& n_Pose) {
    Pose_Mount.setValue(n_Pose);
}

void PlanningObject::onChanged(const Property *prop)
{
    Part::Feature::onChanged(prop);
}

void PlanningObject::setAssembleCenter_toFeatureCenter() {
    Base::Placement t_Center;
    auto ref_Edge = CAD_Utility::getEdgesFromDataSource(tCurvFeature);
    if (ref_Edge.size()) {
        t_Center = CAD_Utility::getCurveCenterPnt(ref_Edge.front());
    }
    else{
        t_Center = CAD_Utility::calculateLinkedFaceCenter(tFaceFeature);
    }
    updateTranslation_Origin2Mount(t_Center);
}


void PlanningObject::Save(Base::Writer &writer) const {
  Part::Feature::Save(writer);
}

void PlanningObject::Restore(Base::XMLReader &reader) {
  Part::Feature::Restore(reader);
}

void PlanningObject::onDocumentRestored() {

}



