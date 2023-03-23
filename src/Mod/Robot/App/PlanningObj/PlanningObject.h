#pragma once
#include <App/GeoFeature.h>
#include <App/PropertyContainer.h>
#include <App/PropertyStandard.h>
#include "Mod/Robot/App/Utilites/FrameObject.h"

#include <Mod/Mesh/App/Mesh.h>
#include <Mod/Part/App/PartFeature.h>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

//using namespace RD;
namespace Robot {
//class rd_RobotLibInterface;
class PlanningObject : public Part::Feature {
  PROPERTY_HEADER(Robot::PlanningObject);

public:
  PlanningObject();
  virtual const char *getViewProviderName() const override {
    return "RobotGui::ViewProviderPlanningObj";
  }
  virtual void Save(Base::Writer &writer) const;
  virtual void Restore(Base::XMLReader &reader);

  void insertIntoCollisionWorld();
  const Base::Placement getCurrentMountPose();
  const Base::Placement getCurrentFramePose();
  // Assemble
  void setMountPose_toFeatureCenter();
  void setFramePose_toFeatureCenter();
  bool changeMountState(const char* device_Name, bool attachTo);

protected:
  virtual void onChanged (const App::Property* prop);
  void onDocumentRestored() override;

public:
  App::PropertyString FilePath_Solid;
  App::PropertyBool isEditing;
  App::PropertyPlacement Trans_O2M;
  App::PropertyPlacement Pose_Mount;
  App::PropertyBool FrameOn;
  App::PropertyPlacement Trans_O2F;

  App::PropertyString AttachedTo;
  App::PropertyLinkSub tCurvFeature;
  App::PropertyLinkSub tFaceFeature;

protected:
  FrameObject *m_ObjectFrame = nullptr;
};
}
