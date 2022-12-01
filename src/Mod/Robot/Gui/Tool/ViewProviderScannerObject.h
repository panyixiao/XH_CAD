// Created by Yixiao 2022-04-24
#pragma once
#include <Inventor/nodes/SoGroup.h>

#include <Gui/ViewProviderGeometryObject.h>
#include <Mod/Part/Gui/ViewProviderExt.h>

#include <Gui/SoFCSelection.h>
#include "ViewProviderToolObject.h"
#include "Mod/Robot/App/Tool/ScannerObject.h"

using namespace Robot;

class SoCoordinate3;
class SoDrawStyle;
class SoLineSet;

namespace RobotGui {

class ViewProviderScannerObject : public ViewProviderToolObject {

  PROPERTY_HEADER(RobotGui::ViewProviderScannerObject);

public:
  ViewProviderScannerObject();
  ~ViewProviderScannerObject();
  // Inherited
  void onChanged(const App::Property* prop);
  void updateData(const App::Property *prop);
  void attach(App::DocumentObject *obj);
  virtual void setDisplayMode(const char *displayModeName);
  virtual std::vector<std::string> getDisplayModes() const;
  bool setEdit(int ModNum);
//  void unsetEdit(int ModNum);
  bool doubleClicked();
protected:
  void updateScanTrianglePose();

private:
  Gui::SoFCSelection    * pcScanRoot;
  SoCoordinate3         * pcCoords;
  SoDrawStyle           * pcDrawStyle;
  SoLineSet             * pcLines;
};
}
