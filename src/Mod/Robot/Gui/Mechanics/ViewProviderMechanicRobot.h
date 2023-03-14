// Created By Yixiao 2023-03-10

#ifndef ROBOT_VIEWPROVIDER_MECHANICROBOT_H
#define ROBOT_VIEWPROVIDER_MECHANICROBOT_H

#include "Mod/Robot/Gui/PreCompiled.h"
#include "Mod/Robot/Gui/Mechanics/ViewProviderMechanicBase.h"


namespace RobotGui
{

class RobotGuiExport ViewProviderMechanicRobot : public ViewProviderMechanicBase
{
    PROPERTY_HEADER(RobotGui::ViewProviderMechanicRobot);

public:
    /// constructor.
    ViewProviderMechanicRobot();
    /// destructor.
    ~ViewProviderMechanicRobot();
    virtual std::vector<App::DocumentObject *> claimChildren(void) const override;
    virtual void onChanged(const App::Property* prop) override;
    void updateData(const App::Property*) override;
    bool setTipPosition(const Base::Placement& new_Pose);
    void setTipPoseByTeach(const Base::Placement& new_Pose);
    bool setEdit(int ModNum) override;
    void unsetEdit(int ModeNum) override;

protected:
    bool updatelinkmeshPoses();
    void DraggerMotionCallback(InteractiveDragger *t_dragger);
    void DraggerFinishCallback(InteractiveDragger *t_dragger);

protected:
    // view provider of the toolshape if set
    App::DocumentObject   * toolShapeObj = nullptr;
    Gui::ViewProvider     * toolShapeVP = nullptr;
};

}

#endif
