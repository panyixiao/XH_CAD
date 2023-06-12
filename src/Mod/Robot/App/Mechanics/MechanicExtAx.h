// Created by Yixiao 2023-06-10

#ifndef ROBOT_MECHANICEXTAX_H
#define ROBOT_MECHANICEXTAX_H

#include "Mod/Robot/App/PreCompiled.h"
#include "MechanicBase.h"
#include "Mod/Robot/App/Mechanics/MechanicRobot.h"
#include "Mod/Robot/App/Tool/ToolObject.h"

namespace Robot
{
class RobotExport MechanicExtAx : public Robot::MechanicBase{
    PROPERTY_HEADER(Robot::MechanicExtAx);

public:
    /// Constructor
    MechanicExtAx(void);
    virtual ~MechanicExtAx();

    // Document Obejct Operations
    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const override{
        return "RobotGui::ViewProviderMechanicBase";
    }
    virtual App::DocumentObjectExecReturn *execute(void) override;
    virtual App::DocumentObjectExecReturn *recompute() override;
    virtual short mustExecute(void) const override;
    virtual PyObject *getPyObject(void);
    virtual void Save (Base::Writer &/*writer*/) const override;
    virtual void Restore(Base::XMLReader &/*reader*/) override;

    // TODO: Add Tool Function Back
    bool setTipPose(const Base::Placement& tip_Pose,
                    CoordOrigin pose_Origin = CoordOrigin::World,
                    Base::Placement origin_Pose = Base::Placement());

    const Base::Placement getToolTipTranslation() const;

    // Poser can attach Tool/Object at the Tip
    void mountMechanicRobot(const char* toolName);
    void dismountMechanicRobot();
//    void mountWorkingObject(const char* obejctName);
//    void dismountWorkingObject();
    void updateMountedRobotPose();
    std::vector<App::DocumentObject*> getChildrenList() const;

public:
//    App::PropertyStringList MountedObjectNames;
    App::PropertyString     MountedRobotName;
//    App::PropertyString     AssembledToolName;
    App::PropertyInteger    RatedLoad;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    void onDocumentRestored() override;

private:
//    std::vector<Robot::PlanningObject*> m_MountedObjects;
//    Robot::PlanningObject* m_MountedObjectPtr = nullptr;
//    Robot::ToolObject* m_AssembledToolPtr = nullptr;
    Robot::MechanicRobot* m_MountedRobotPtr = nullptr;
};

} //namespace Robot


#endif
