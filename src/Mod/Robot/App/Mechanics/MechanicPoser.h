// Created by Yixiao 2023-03-10

#ifndef ROBOT_MECHANICPOSER_H
#define ROBOT_MECHANICPOSER_H

#include "Mod/Robot/App/PlanningObj/PlanningObject.h"
#include "Mod/Robot/App/Tool/ToolObject.h"
#include "MechanicBase.h"

namespace Robot
{
class RobotExport MechanicPoser : public Robot::MechanicBase{
    PROPERTY_HEADER(Robot::MechanicPoser);

public:
    /// Constructor
    MechanicPoser(void);
    virtual ~MechanicPoser();

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
    void assembleTool(const char* toolName);
    void disassembleTool();
    void mountWorkingObject(const char* obejctName);
    void dismountWorkingObject();

    void udpateLoadPosition();
    std::vector<App::DocumentObject*> getChildrenList() const;

public:
    App::PropertyString    MountedWorkingObj;
    App::PropertyString    AssembledTool;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    void onDocumentRestored() override;

private:
    Robot::PlanningObject* m_MountedWorkingObjPtr = nullptr;
    Robot::ToolObject* m_AssembledToolPtr = nullptr;
};

} //namespace Robot


#endif
