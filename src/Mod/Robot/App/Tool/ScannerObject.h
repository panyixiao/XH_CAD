// Created by Yixiao 2022-06-15

#ifndef ROBOT_SCANNNERTOOLOBJECT_H
#define ROBOT_SCANNNERTOOLOBJECT_H

#include <App/GeoFeature.h>
#include <Eigen/Geometry>
#include <Mod/Part/App/PartFeature.h>

#include "Mod/Robot/App/Utilites/FrameObject.h"

#include "ToolObject.h"

namespace Robot {

class ScannerObject : public ToolObject {
  PROPERTY_HEADER(Robot::ScannerObject);
public:
  ScannerObject();
  virtual ~ScannerObject();
  const char *getViewProviderName(void) const {
    return "RobotGui::ViewProviderScannerObject";
  }

//  // Tool Operation
//  bool attachObject(App::DocumentObject *t_obj);
//  bool detachObjcet(App::DocumentObject *t_obj);

  virtual void Save(Base::Writer &writer) const;
  virtual void Restore(Base::XMLReader &reader);

  App::PropertyFloat    ScanDistance;
  App::PropertyFloat    ScanAmplitute;
  App::PropertyBool     LaserOn;
protected:
  /// get called by the container when a property has changed
  virtual void onChanged (const App::Property* prop);
  void onDocumentRestored() override;
  void updateTip2FrontTrans();
};
}

#endif // ROBOT_TOOLOBJECT_H
