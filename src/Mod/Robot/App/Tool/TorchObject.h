// Created by Yixiao 2022/04/24
#ifndef ROBOT_TORCHTOOLOBJECT_H
#define ROBOT_TORCHTOOLOBJECT_H

#include <App/GeoFeature.h>
#include <Eigen/Geometry>
#include <Mod/Part/App/PartFeature.h>

//#include "Mod/Robot/App/Utilites/FrameObject.h"

#include "ToolObject.h"

namespace Robot {

class TorchObject : public ToolObject {
  PROPERTY_HEADER(Robot::TorchObject);
public:
  TorchObject();
  virtual ~TorchObject();
  const char *getViewProviderName(void) const {
    return "RobotGui::ViewProviderTorchObject";
  }

  virtual void Save(Base::Writer &writer) const;
  virtual void Restore(Base::XMLReader &reader);


protected:
  /// get called by the container when a property has changed
  virtual void onChanged (const App::Property* prop);
  void onDocumentRestored() override;

public:
  App::PropertyBool SparkOn;
};
}

#endif // ROBOT_TOOLOBJECT_H
