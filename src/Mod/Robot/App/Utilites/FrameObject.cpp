// Created by Yixiao 2022-04-24

#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "FrameObject.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Command.h>
#include "Mod/Robot/App/PlanningObj/PlanningObject.h"

using namespace Robot;

PROPERTY_SOURCE(Robot::FrameObject, App::GeoFeature)

FrameObject::FrameObject() {
  ADD_PROPERTY(ParentFrameID, (""));
  ADD_PROPERTY(FrameID, (""));
  ADD_PROPERTY(Visible, (false));
  ADD_PROPERTY(ChildrenList, (""));
  ADD_PROPERTY(Owner, (""));
  ParentFrameID.setStatus(App::Property::Status::ReadOnly, true);
  FrameID.setStatus(App::Property::Status::ReadOnly, true);
  ChildrenList.setStatus(App::Property::Status::ReadOnly, true);
  Owner.setStatus(App::Property::Status::ReadOnly, true);
}

FrameObject::~FrameObject() {}

//bool FrameObject::registerFrameInLibrary(
//    const string frameID/*, sptr<rd_RobotLibInterface> libInterface*/,
//    const string P_FrameID) {
//  updateFrameID(frameID);
////  if (!libInterface) {
////    string Msg = string("RD_Frame: Libinterface is nullptr, "
////                        "registerFrameInLibrary() failed!\n");
////    Base::Console().Error(Msg.c_str());
////    return false;
////  }
////  this->setParentFrame(IRD_Planning_State::gPlanningFrame);
////  this->setName(frameID);
////  this->setPoseInParentFrame(
////      DS_Utility::PlacementToPoseStamped(Placement.getValue()).pose);
////  if (!libInterface->addFrame(*this)) {
////    string Msg = string("RD_Frame: adding Frame to Library Failed, "
////                        "registerFrameInLibrary() failed!\n");
////    Base::Console().Error(Msg.c_str());
////    return false;
////  }
////  if (P_FrameID.empty())
////    updateParentFrameID(libInterface->getPlanningFrameName());
////  else
////    updateParentFrameID(P_FrameID);
////  m_LibInterfacePtr = libInterface;
//  registered = true;
//  string Msg =
//      string("RD_Frame: ") + frameID + string(" registered and created!\n");
//  Base::Console().Message(Msg.c_str());
//  return true;
//}

const string FrameObject::getFrameNameInDocument() const {
  auto frameName = getNameInDocument();
  if (frameName)
    return string(frameName);
  Base::Console().Message("Cant get name from Document return FrameID");
  return FrameID.getStrValue();
}

// Get called when placement update is triggered from ui
bool FrameObject::updateFramePlacement(const Base::Placement &newPlacement) {
//  if (m_LibInterfacePtr == nullptr) {
//    Base::Console().Error(
//        "RD_Frame: Libinterface is nullptr, updateFramePlacement() failed\n");
//    return false;
//  }
//  if (!m_LibInterfacePtr->editFrame(*this, ParentFrameID.getValue(),
//                                    newPlacement)) {
//    Base::Console().Error("RD_Frame: Failed to change FramePose in library, "
//                          "updateFramePlacement() failed!\n");
//    return false;
//  }
  Robot::DS_Utility::setValueIfDifferent(Placement, newPlacement);
  updateChildrenTransformation();
  return true;
}

// Get Called when placement upadat is triggered passively (by parent frame)
void FrameObject::updateFramePlacement_FromLib() {
//  if (!m_LibInterfacePtr) {
//    Base::Console().Error("RD_Frame: Libinterface is nullptr, Update Placement "
//                          "from Lib Failed!\n");

//    return;
//  }
//  auto poseMap = m_LibInterfacePtr->getObjectPoseMap();
//  auto targetName = getFrameNameInDocument() + string("_center");
//  auto objIter = poseMap.find(targetName);
//  if (objIter != poseMap.end()) {
//    auto const &newPose = objIter->second;
//    if (RD_CAD_Utility::DS_Utility::setValueIfDifferent(Placement, newPose)) {
//      updateChildrenTransformation();
//    }
//  } else {
//    std::cout << "Can't find " << targetName << " in poseMap" << std::endl;
//  }
}

const Base::Placement FrameObject::getPlacement() const {
  return Placement.getValue();
}

void FrameObject::setObjectParentFrame(App::DocumentObject *t_obj) {
//  if (t_obj == nullptr)
//    return;
//  if (t_obj->isDerivedFrom(Robot::PlanningObject::getClassTypeId())) {
//    auto t_planningObj = dynamic_cast<Robot::PlanningObject *>(t_obj);
////    t_planningObj->setParentFrame(FrameID.getStrValue());
//  } else if (t_obj->isDerivedFrom(Robot::FrameObject::getClassTypeId())) {
//    auto t_frameObj = dynamic_cast<Robot::FrameObject *>(t_obj);
//    t_frameObj->setNewParentFrame(FrameID.getStrValue());
//    t_frameObj->ParentFrameID.setValue(FrameID.getStrValue());
//  }
}

void FrameObject::insertToTargetFrame(const string &parentFrameID) {
  if (parentFrameID.empty())
    return;
  auto t_doc = getDocument();
  if (!t_doc)
    return;
  auto parentFramePtr = dynamic_cast<Robot::FrameObject *>(
      t_doc->getObject(parentFrameID.c_str()));
  if (parentFramePtr) {
    parentFramePtr->addObjectIntoFrame(this);
//    if (m_LibInterfacePtr)
//      m_LibInterfacePtr->editFrame(*this, parentFrameID, Placement.getValue());
  }
}

void FrameObject::setNewParentFrame(const string &parent) {
//  setParentFrame(parent);
  updateParentFrameID(parent);
}

bool FrameObject::addObjectIntoFrame(App::DocumentObject *t_obj) {
  if (!t_obj)
    return false;
  auto objIter =
      std::find_if(m_ChildrenObjects.begin(), m_ChildrenObjects.end(),
                   [t_obj](App::DocumentObject *objInVec) {
                     return !strcmp(t_obj->getNameInDocument(),
                                    objInVec->getNameInDocument());
                   });
  if (objIter != m_ChildrenObjects.end())
    return false;

  if (t_obj->isDerivedFrom(Robot::FrameObject::getClassTypeId())) {
    auto t_frameObj = dynamic_cast<Robot::FrameObject *>(t_obj);
    setObjectParentFrame(t_frameObj);
    m_ChildrenObjects.push_back(t_frameObj);
  }

  refreshChildrenList();
  return true;
}

bool FrameObject::removeObjectFromFrame(App::DocumentObject *t_obj) {
  auto objIter =
      std::find_if(m_ChildrenObjects.begin(), m_ChildrenObjects.end(),
                   [t_obj](App::DocumentObject *objInVec) {
                     return !strcmp(t_obj->getNameInDocument(),
                                    objInVec->getNameInDocument());
                   });
  if (objIter == m_ChildrenObjects.end()) {
    Base::Console().Message(
        "Can't find target in Frame Children List, Remove Failed!\n");
    std::cout << "Can't find target in Frame Children List, Remove Failed!"
              << std::endl;
    return false;
  }
  m_ChildrenObjects.erase(objIter);
  refreshChildrenList();
  resetChildParentFrame(t_obj);
  return true;
}

const vector<App::DocumentObject *> &FrameObject::getObjectsInFrame() const {
  return m_ChildrenObjects;
}

void FrameObject::resetParentFrame() {
//  if (m_LibInterfacePtr)
//    updateParentFrameID(m_LibInterfacePtr->getPlanningFrameName());
//  else
    updateParentFrameID(string(""));
}

void FrameObject::updateChildrenTransformation() {
  for (auto t_obj : m_ChildrenObjects) {
    if (t_obj->isDerivedFrom(Robot::PlanningObject::getClassTypeId())) {
      auto t_PlanningObj = dynamic_cast<Robot::PlanningObject *>(t_obj);
//      t_PlanningObj->updateInteractionCenter_FromLib();
    }
    else if (t_obj->isDerivedFrom(Robot::FrameObject::getClassTypeId())) {
      auto t_FrameObj = dynamic_cast<Robot::FrameObject *>(t_obj);
      t_FrameObj->updateFramePlacement_FromLib();
      t_FrameObj->updateChildrenTransformation();
    }
  }
}

void FrameObject::resetChildParentFrame(App::DocumentObject *t_obj) {
  if (t_obj == nullptr)
    return;
    if (t_obj->isDerivedFrom(Robot::FrameObject::getClassTypeId())) {
    auto t_FrameObj = dynamic_cast<Robot::FrameObject *>(t_obj);
    t_FrameObj->resetParentFrame();
  }
}

void FrameObject::resetChildrenParentFrame() {
  for (auto t_obj : m_ChildrenObjects) {
    resetChildParentFrame(t_obj);
  }
}

void FrameObject::refreshChildrenList() {
  vector<string> childrenObj_names;
  for (auto objPtr : m_ChildrenObjects) {
    childrenObj_names.push_back(string(objPtr->getNameInDocument()));
  }
  updateChildrenList(childrenObj_names);
}

void FrameObject::onDocumentRestored() {
  restoreChildrenVec();
  refreshChildrenList();
}

void FrameObject::restoreChildrenVec() {
  auto childrenNames = ChildrenList.getValues();
  for (auto childName : childrenNames) {
    auto childPtr = getDocument()->getObject(childName.c_str());
    if (childPtr)
      m_ChildrenObjects.push_back(childPtr);
  }
}

void FrameObject::updateFrameOwner(const string &owner_name) {
  Owner.setStatus(App::Property::Status::ReadOnly, false);
  Owner.setValue(owner_name.c_str());
  Owner.setStatus(App::Property::Status::ReadOnly, true);
}

void FrameObject::updateParentFrameID(const string &frameID) {
  //  ParentFrameID.setStatus(App::Property::Status::ReadOnly, false);
  ParentFrameID.setValue(frameID);
  //  ParentFrameID.setStatus(App::Property::Status::ReadOnly, true);
}

void FrameObject::updateFrameID(const string &frameID) {
  FrameID.setStatus(App::Property::Status::ReadOnly, false);
  FrameID.setValue(frameID.c_str());
  FrameID.setStatus(App::Property::Status::ReadOnly, true);
}

void FrameObject::updateChildrenList(const std::vector<string> &childrenList) {
  ChildrenList.setStatus(App::Property::Status::ReadOnly, false);
//  ChildrenList.
  ChildrenList.setValues(childrenList);
  ChildrenList.setStatus(App::Property::Status::ReadOnly, true);
}

bool FrameObject::createFrameObject(string const &frameName) {
  Gui::Command::openCommand("Insert Tool Frame");
  Gui::Command::doCommand(
      Gui::Command::Doc,
      "App.activeDocument().addObject(\"Robot::FrameObject\",\"%s\")",
      frameName.c_str());
  Gui::Command::commitCommand();
  return true;
}

bool FrameObject::setupFrame(string const &frameName,
                         App::DocumentObject *obj,
                         string &frameNameProp,
                         Robot::FrameObject *&framePtr) {

  // Make sure there is only 1 frame for each object
  if (frameNameProp.empty()) {
    createFrameObject(frameName);
    frameNameProp = frameName;
  }
  if (framePtr == nullptr) {
    auto t_obj = obj->getDocument();
    if (t_obj) {
      framePtr = dynamic_cast<Robot::FrameObject *>(
          t_obj->getObject(frameNameProp.c_str()));
    }
  }
  framePtr->updateFrameOwner(string(obj->getNameInDocument()));
  return true;
}

const string FrameObject::getWorldPlanningFrame() const {
//  if (m_LibInterfacePtr == nullptr)
//    return string("");
//  return m_LibInterfacePtr->getPlanningFrameName();
    return string("");
}
