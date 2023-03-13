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
  void insertIntoCollisionWorld();
  void updateTranslation_Origin2Mount(const Base::Placement &t_Translation);
  const Base::Placement getCurrentMountPose();
  void updateObjectMountPose(const Base::Placement &n_Pose);
  void setAssembleCenter_toFeatureCenter();

  virtual void Save(Base::Writer &writer) const;
  virtual void Restore(Base::XMLReader &reader);

  /// object frame
  void setupObjectFrame(std::shared_ptr<FrameObject> t_frame = nullptr);
  bool setupObjectFrameProperty();
  void restoreObjectFrame();

protected:
  virtual void onChanged (const App::Property* prop);
  void onDocumentRestored() override;

public:
//  bool restoringObject = false;
  App::PropertyPlacement Translation_O2M;
  App::PropertyPlacement Pose_Mount;

  App::PropertyString ObjectName;
  App::PropertyString FilePath;

  App::PropertyLinkSub tCurvFeature;
  App::PropertyLinkSub tFaceFeature;

protected:
  FrameObject *m_ObjectFrame = nullptr;
};
}
