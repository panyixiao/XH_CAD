// Created by Yixiao 2023-03-10


#ifndef ROBOT_MECHANICROBOT_H
#define ROBOT_MECHANICROBOT_H

#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include <App/PropertyLinks.h>

#include "KinematicModel.h"
#include "Mod/Robot/App/Tool/ToolObject.h"
#include "Mod/Robot/App/Mechanics/MechanicBase.h"
#include "Mod/Robot/App/Mechanics/MechanicPoser.h"

namespace Robot
{
class RobotExport MechanicRobot : public Robot::MechanicBase
{
    PROPERTY_HEADER(Robot::MechanicRobot);

public:
    /// Constructor
    MechanicRobot(void);
    virtual ~MechanicRobot();

    // Document Obejct Operations
    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderMechanicRobot";
    }
    virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

    void moveToSelectedFaceCenter();
    bool setRobotPose(const RobotPose &t_Pose);
    const RobotPose getRobotPose(const CordType& t_Type) const;


    virtual bool setJointAngle(const size_t jntID, float jntAngle) override;
    void setTipPoseByDraggerPose(const Base::Placement& n_DraggerPose);
    void setTipPoseByDiff(const Base::Placement &movement);
    bool setRobotTipPose(const Base::Placement& tip_Pose,
                         CoordOrigin pose_Origin = CoordOrigin::World,
                         Base::Placement origin_Pose = Base::Placement());

    const Base::Placement getTeachDraggerPose() const;
    const Base::Placement getToolTipTranslation() const;
    const Base::Placement getToolFrameTrans(const uint ToolID) const;
    const Base::Placement getCurrentTipPose(const CoordOrigin &ref_Origin) const;
    const Base::Placement getCurrentFlanPose(const CoordOrigin &ref_Origin) const;

    // Tool Setting
    bool targetToolAssemebled(const ToolType& t_Type);
    bool installTool(const char* tool_Name);
    void uninstallTool(const char *tool_Name);
    void updateAssembledToolPose();
    void setCurrentToolType(const ToolType& t_Type);
    void setCurrentToolActive(bool activated = false);
    ToolType getCurrentTool() const;

    std::vector<App::DocumentObject*> getChildrenList() const;

    const Base::Placement getCurrentBasePose() const;
    const Base::Placement getSelectedFeatureCenter() const;

    void setTeachCoordType(const TeachCoord& t_coord);
    const TeachCoord& getCurrentTeachCoord() const {
        return m_TeachCoord;
    }
    // 变位机/外部轴
    uint getAvailablePoserAxisNumber() const;
    Robot::MechanicPoser* getTargetLinkedPoser(const std::string poserName);
    QStringList getLinkedPoserInfo(const string &t_PoserName) const;
    QStringList getLinkedExtAxInfo(const string &t_ExtAxName) const;

public:
    App::PropertyPlacement   Pose_Flan;
    App::PropertyIntegerList ArmConfiguration;  // Wrist | Forearm | Elbow
    App::PropertyBool        EnableArmConfiguration;
    // Connected ExtAx
    App::PropertyString      LinkedExtAxName;
    App::PropertyStringList  LinkedPoserNames;
    // IK Trigger
    App::PropertyInteger     TeachCoordIndex;
    // Tool
    App::PropertyInteger     CurrentToolIndex;
    App::PropertyBool        InteractiveDraggerOn;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop) override;
    void onDocumentRestored() override;
    void updateRobotConfiguration();
private:
    TeachCoord m_TeachCoord = TeachCoord::RobotBase;
    // Robot::MechanicExtAx* m_LinkedExtAxPtr;
    //
    std::map<std::string, Robot::MechanicPoser*> m_LinkedPosers;
    //
    std::map<uint, Robot::ToolObject*> m_AssembledTools;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H
