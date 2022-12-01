#ifndef VIEWPROVIDERFRAMEOBJECT_H
#define VIEWPROVIDERFRAMEOBJECT_H

#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"
#include <Gui/ViewProviderGeometryObject.h>
//#include <Mod/Robot/App/R_Frame.h>
//#include <Mod/RD_TaskManager/App/rd_Frame.h>
//#include <RD_DataStructure/RD_STL/RD_Vector.h>
#include "Mod/Robot/App/Utilites/FrameObject.h"

class SoFCCSysDragger;
class SoCoordinate3;
class SoDrawStyle;
//using namespace RD;
namespace RobotGui {
class ViewProviderFrameObject : public Gui::ViewProviderGeometryObject {
  PROPERTY_HEADER(RD_SetupGui::ViewProvider_RD_Frame);

public:
  ViewProviderFrameObject();
  ~ViewProviderFrameObject();
  void attach(App::DocumentObject *pcObject);
  void setDisplayMode(const char *ModeName);
  std::vector<std::string> getDisplayModes() const;
  void updateData(const App::Property *prop);
  virtual void onChanged(const App::Property *prop);
  bool setEdit(int ModNum);
  void unsetEdit(int ModNum);
  virtual bool canDragObjects() const { return true; }
  virtual bool canDropObjects() const { return true; }
  virtual void dragObject(App::DocumentObject *t_obj) override;
  virtual void dropObject(App::DocumentObject *t_obj) override;
  virtual bool onDelete(const std::vector<std::string> &subNames);
  virtual std::vector<App::DocumentObject *> claimChildren(void) const override;
  void updatePlacement(const Base::Placement &newPlacement);

protected:
  void setupDragger(bool enable);

protected:
  Gui::SoFCSelection *m_frameOriginRoot = nullptr;
  std::unique_ptr<InteractiveDragger> m_OperationDragger;
  Robot::FrameObject *m_Frame = nullptr;

private:
  bool draggerIsMoving = false;
};
}

#endif // VIEWPROVIDER_RD_FRAME_H
