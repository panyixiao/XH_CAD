// Created by Yixiao 2022-06-15
#pragma once
#include <Inventor/nodes/SoGroup.h>

#include <Gui/ViewProviderGeometryObject.h>
#include <Mod/Part/Gui/ViewProviderExt.h>
#include <Gui/SoFCSelection.h>

#include "Mod/Robot/App/Tool/TorchObject.h"
#include "ViewProviderToolObject.h"

using namespace Robot;
namespace RobotGui {

class ViewProviderTorchObject : public ViewProviderToolObject {

  PROPERTY_HEADER(RobotGui::ViewProviderTorchObject);

public:
  ViewProviderTorchObject();
  ~ViewProviderTorchObject();
  // Inherited
  void onChanged(const App::Property* prop);
  void updateData(const App::Property *prop);
  void attach(App::DocumentObject *obj);
  virtual void setDisplayMode(const char *displayModeName);
  virtual std::vector<std::string> getDisplayModes() const;
  bool setEdit(int ModNum);
//  void unsetEdit(int ModNum);
  bool doubleClicked();


private:
  Robot::TorchObject *m_TorchPtr = nullptr;
};
}
