#pragma once

#include <Gui/ViewProviderGeometryObject.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

//#include "Mod/RD_Setup/App/rd_PlanningObject.h"
#include "Mod/Robot/App/PlanningObj/PlanningObject.h"

#include <Gui/SoFCSelection.h>
#include <Mod/Part/Gui/SoBrepEdgeSet.h>
#include <Mod/Part/Gui/SoBrepFaceSet.h>
#include <Mod/Part/Gui/SoBrepPointSet.h>
#include <Mod/Part/Gui/ViewProviderExt.h>

using namespace PartGui;

namespace RobotGui {
class ViewProviderPlanningObj : public PartGui::ViewProviderPartExt {
  PROPERTY_HEADER(RobotGui::ViewProviderPlanningObj);

public:
  ViewProviderPlanningObj();
  ~ViewProviderPlanningObj();
  virtual void attach(App::DocumentObject *pcObj);
  virtual void setDisplayMode(const char *displayModeName);
  virtual std::vector<std::string> getDisplayModes() const;
  virtual void updateData(const App::Property *prop);
  virtual void onChanged(const App::Property *prop);
  virtual bool doubleClicked();
  void updateCenterPlacement(const Base::Placement &newPlacement);
  bool setEdit(int ModNum = 0);
  void unsetEdit(int ModNum);
  bool onDelete(const std::vector<std::string> &subNames);
  virtual std::vector<App::DocumentObject *> claimChildren(void) const override;

protected:
  void updateRenderStatus(bool colliding);

private:
  InteractiveDragger *m_dragger = nullptr;
  App::Document* m_DocPtr = nullptr;
//  Robot::PlanningObject* m_PlanningObj = nullptr;
};
}
