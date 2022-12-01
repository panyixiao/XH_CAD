// Created by Yixiao 20220423

#ifndef FRAMEOBJECT_H
#define FRAMEOBJECT_H

#include <App/DocumentObject.h>
#include <App/GeoFeature.h>
#include <App/Placement.h>
#include <Base/Placement.h>

//using namespace RD;
namespace Robot {

enum class CordType{
    PCS = 0,
    MCS,
    ACS,
    TCS,
    VCS,
    WCS // World Cord System
};

// TODO: Re-Factory, be same with Robot::CordType (RobotWaypoint.h)
enum class CoordOrigin{
    World = 0,
    Robot,
    Flan,
    Object
};

//class rd_RobotLibInterface;
class FrameObject : public App::GeoFeature {
  PROPERTY_HEADER(Robot::FrameObject);

public:
  FrameObject();
  virtual ~FrameObject();
//  virtual bool registerFrameInLibrary(const string frameID,
//                                      sptr<rd_RobotLibInterface> libInterface,
//                                      const string P_FrameID = string());

  virtual const char *getViewProviderName() const override {
    return "RobotGui::ViewProviderFrameObject";
  }

  virtual const std::string getFrameNameInDocument() const;

  static bool createFrameObject(std::string const &frameName);
  static bool setupFrame(std::string const &frameName,
                         App::DocumentObject *obj,
                         std::string &frameNameProp,
                         FrameObject *&framePtr);
  const std::string getWorldPlanningFrame() const;

  virtual bool updateFramePlacement(const Base::Placement &newPlacement);
  void updateFramePlacement_FromLib();

  const Base::Placement getPlacement() const;
  void setObjectParentFrame(App::DocumentObject *t_obj);
  void insertToTargetFrame(const std::string &parentFrameID);
  virtual void resetParentFrame();
  // Object operation
  void setNewParentFrame(const std::string &parentFrameID);
  bool addObjectIntoFrame(App::DocumentObject *t_obj);
  bool removeObjectFromFrame(App::DocumentObject *t_obj);
  const std::vector<App::DocumentObject *> &getObjectsInFrame() const;
  void updateChildrenTransformation();
  void resetChildParentFrame(App::DocumentObject *t_obj);
  void resetChildrenParentFrame();
  void refreshChildrenList();
  //  bool frameRegisterd() { return registered; }

protected:
  void onDocumentRestored() override;
  void restoreChildrenVec();
  void updateFrameOwner(const std::string &owner_name);
  void updateParentFrameID(const std::string &frameID);
  void updateFrameID(const std::string &frameID);
  void updateChildrenList(const std::vector<std::string> &childrenList);

public:
  App::PropertyString Owner;
  App::PropertyString ParentFrameID;
  App::PropertyString FrameID;
  App::PropertyStringList ChildrenList;
  App::PropertyBool Visible;

protected:
//  sptr<rd_RobotLibInterface> m_LibInterfacePtr;
  std::vector<App::DocumentObject *> m_ChildrenObjects;
  bool registered = false;
};
}

#endif // RD_FRAME_H
