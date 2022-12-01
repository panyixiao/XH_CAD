// Created By Yixiao 2022-08-14

#ifndef ROBOT_VIEWPROVIDERMECHANICGROUP_H
#define ROBOT_VIEWPROVIDERMECHANICGROUP_H

#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Gui/ViewProviderGeometryObject.h>
#include <Gui/SoFCSelection.h>
#include <Gui/SoFCCSysDragger.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/Utilites/MeshUtility.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"
#include "ViewProviderRobot6AxisObject.h"
#include "ViewProviderMechanicDevice.h"

typedef std::function<void()> RobotVP_CallBack;

namespace RobotGui
{

class RobotGuiExport ViewProviderMechanicGroup : public Gui::ViewProviderGeometryObject
{
    PROPERTY_HEADER(RobotGui::ViewProviderMechanicGroup);

public:
    /// constructor.
    ViewProviderMechanicGroup();

    /// destructor.
    ~ViewProviderMechanicGroup();

    App::PropertyInteger CurrentInteractiveCoord;

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
    bool generateLinkMeshNodes(const char *FileName);
    bool generateLinkMeshNodes_fromURDF(const char *FileName);
    bool updatelinkmeshPoses(const Base::Placement &basePose);
    bool updateTipPanelCallbackRegistered();

public:
    App::PropertyBool       EnableSelection;

protected:
    // ROBOT OBJECT
    Gui::SoFCSelection    *  pcRoot = nullptr;
    SoTransform           *  pcTcpTransform = nullptr;
    // LINKS
    float                    mesh_sf = 1000;
    SoGroup               *  m_LinkMeshGroup = nullptr;
    std::vector<std::string> m_LinkNames;

    // ROBOT TIPS
    SoGroup               * m_TcpRoot = nullptr;
    InteractiveDragger    * pcDragger = nullptr;

    // view provider of the toolshape if set
    App::DocumentObject   * toolShapeObj = nullptr;
    Gui::ViewProvider     * toolShapeVP = nullptr;

    RobotVP_voidCallBack        callBack_UpdatePanelWidgets = nullptr;
    RobotVP_boolCallBack        callBack_ConnectedToStation = nullptr;
};

} //namespace RobotGui


#endif // ROBOT_VIEWPROVIDERROBOTOBJECT_H
