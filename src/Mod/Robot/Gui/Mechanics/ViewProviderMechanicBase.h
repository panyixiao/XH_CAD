// Created By Yixiao 2023-03-10

#ifndef ROBOT_VIEWPROVIDER_MECHANIC_H
#define ROBOT_VIEWPROVIDER_MECHANIC_H

#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Gui/ViewProviderGeometryObject.h>
#include <Gui/SoFCSelection.h>
#include <Gui/SoFCCSysDragger.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/Utilites/MeshUtility.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

namespace RobotGui
{

class RobotGuiExport ViewProviderMechanicBase : public Gui::ViewProviderGeometryObject
{
    PROPERTY_HEADER(RobotGui::ViewProviderMechanicBase);

public:
    /// constructor.
    ViewProviderMechanicBase();
    /// destructor.
    ~ViewProviderMechanicBase();

    void attach(App::DocumentObject *pcObject) override;
    void setDisplayMode(const char* ModeName) override;
    std::vector<std::string> getDisplayModes() const override;

    void updateData(const App::Property*) override;
    virtual void onChanged(const App::Property* prop) override;

    /// for simulation without changing the document:
    bool setTipPosition(const Base::Placement& new_Pose);
    void setTipPoseByTeach(const Base::Placement& new_Pose);

    virtual bool doubleClicked() override;
    bool setEdit(int ModNum) override;
    void unsetEdit(int ModeNum) override;
    virtual std::vector<App::DocumentObject *> claimChildren(void) const override;

protected:
    void DraggerMotionCallback(InteractiveDragger *t_dragger);
    void DraggerFinishCallback(InteractiveDragger *t_dragger);
    bool generateLinkMeshNodes_fromURDF(const std::string& filePath_urdf);
    bool updatelinkmeshPoses();
    bool callbackRegistered();

protected:
    Gui::SoFCSelection    *  pcRoot = nullptr;
    SoTransform           *  pcTcpTransform = nullptr;
    // LINKS
    float                    mesh_sf = 1000;
    SoGroup               *  m_LinkMeshs = nullptr;
    std::vector<std::string> m_LinkNames;
    // TIPS
    SoGroup               * m_TcpRoot = nullptr;
    InteractiveDragger    * pcDragger = nullptr;
    // view provider of the toolshape if set
    App::DocumentObject   * toolShapeObj = nullptr;
    Gui::ViewProvider     * toolShapeVP = nullptr;

    std::function<void()> callBack_UpdatePanelWidgets = nullptr;
};

} //namespace RobotGui


#endif
