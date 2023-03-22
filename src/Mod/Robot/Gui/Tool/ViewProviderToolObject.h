// Created by Yixiao 2022-04-24
#pragma once
#include <Inventor/nodes/SoGroup.h>

#include <Gui/ViewProviderGeometryObject.h>
#include <Mod/Part/Gui/ViewProviderExt.h>

#include <Gui/SoFCSelection.h>
#include "Mod/Robot/App/Tool/ToolObject.h"

using namespace Robot;
namespace RobotGui {

class ViewProviderToolObject : public PartGui::ViewProviderPartExt {

  PROPERTY_HEADER(RobotGui::ViewProviderToolObject);

public:
  ViewProviderToolObject();
  ~ViewProviderToolObject();
  // Inherited
  void updateData(const App::Property *prop);
  void attach(App::DocumentObject *obj);
  virtual void setDisplayMode(const char *displayModeName);
  virtual std::vector<std::string> getDisplayModes() const;
  bool setEdit(int ModNum);
//  void unsetEdit(int ModNum);
  bool doubleClicked();
  bool onDelete(const std::vector<std::string> &subNames);
  std::vector<App::DocumentObject *> claimChildren() const;

private:
//  Robot::ToolObject *m_tool = nullptr;
};
}
