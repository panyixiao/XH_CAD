// Created By Yixiao 2023-03-10

#ifndef ROBOT_VIEWPROVIDER_MECHANIC_H
#define ROBOT_VIEWPROVIDER_MECHANIC_H

#ifndef _PreComp_
# include <Inventor/SoDB.h>
# include <Inventor/SoInput.h>
# include <Inventor/SbVec3f.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/nodes/SoTransform.h>
# include <Inventor/nodes/SoSphere.h>
# include <Inventor/nodes/SoRotation.h>
# include <Inventor/actions/SoSearchAction.h>
# include <Inventor/draggers/SoJackDragger.h>
# include <Inventor/draggers/SoTrackballDragger.h>
# include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <QFile>
#include <QFileInfo>
#endif

#include <Gui/Application.h>
#include <Gui/Control.h>
#include <Gui/Command.h>
#include <Gui/ViewProviderGeometryObject.h>
#include <Gui/SoFCSelection.h>
#include <Gui/SoFCCSysDragger.h>
#include <Base/Placement.h>
#include "TaskDlgMechanicControl.h"
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
//    // view provider of the toolshape if set
//    App::DocumentObject   * toolShapeObj = nullptr;
//    Gui::ViewProvider     * toolShapeVP = nullptr;

    std::function<void()> callBack_UpdatePanelWidgets = nullptr;
};

} //namespace RobotGui


#endif
