// Created By Yixiao 2022-08-08

#ifndef ROBOT_VIEWPROVIDERPOSITIONER_H
#define ROBOT_VIEWPROVIDERPOSITIONER_H

#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Gui/ViewProviderGeometryObject.h>
#include <Gui/SoFCSelection.h>
#include <Gui/SoFCCSysDragger.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/Utilites/MeshUtility.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

namespace RobotGui
{

class RobotGuiExport ViewProviderMechanicDevice : public Gui::ViewProviderGeometryObject
{
    PROPERTY_HEADER(RobotGui::ViewProviderMechanicDevice);

public:
    /// constructor.
    ViewProviderMechanicDevice();

    /// destructor.
    ~ViewProviderMechanicDevice();

    void attach(App::DocumentObject *pcObject);
    void setDisplayMode(const char* ModeName);
    std::vector<std::string> getDisplayModes() const;

    void updateData(const App::Property*);
    virtual void onChanged(const App::Property* prop);

    /// for simulation without changing the document:
    bool setTipPosition(const Base::Placement& new_Pose);
    void setTipPoseByTeach(const Base::Placement& new_Pose);

    virtual bool doubleClicked();
    bool setEdit(int ModNum);
    void unsetEdit(int ModeNum);    
    virtual std::vector<App::DocumentObject *> claimChildren(void) const override;

protected:
    void DraggerMotionCallback(InteractiveDragger *t_dragger);
    void DraggerFinishCallback(InteractiveDragger *t_dragger);
    bool generateLinkMeshNodes_fromFile(const char *FileName);
    bool updatelinkmeshPoses();
    bool callbackRegistered();

protected:
    // ROBOT OBJECT
    Gui::SoFCSelection    *  pcRoot = nullptr;
    SoTransform           *  pcTcpTransform = nullptr;
    // ROBOT LINKS
    float                    mesh_sf = 1000;
    SoGroup               *  m_LinkMeshGroup = nullptr;
    std::vector<std::string> m_LinkNames;

    // ROBOT TIPS
    SoGroup               * m_TcpRoot = nullptr;
    InteractiveDragger    * pcDragger = nullptr;

    // view provider of the toolshape if set
    App::DocumentObject   * toolShapeObj = nullptr;
    Gui::ViewProvider     * toolShapeVP = nullptr;

    std::function<void()> callBack_UpdatePanelWidgets = nullptr;
};

} //namespace RobotGui


#endif // ROBOT_VIEWPROVIDERROBOTOBJECT_H
