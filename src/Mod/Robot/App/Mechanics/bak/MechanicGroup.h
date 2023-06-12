// Created by Yixiao 2022-08-12

#ifndef ROBOT_MECHANICGROUP_H
#define ROBOT_MECHANICGROUP_H

#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include <App/PropertyLinks.h>

#include "Robot6AxisObject.h"
#include "MechanicDevice.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/Gui/Comm/MechanicGroupCommManager.h"

///
/// TODO : Refactory class Robot6AxisObject and WeldingPositionerObject,
/// Let them both inherited from a bass class called MechanicalBody, in that
/// case the MechanicGroup can be simplifed
///

namespace Robot
{
enum class GroupConfig{
    SingleRobot = 0,
    SingleRobot_2AxisPoser,
    SingleRobot_MultiAxisPoser,
    SingleRobot_ExtAxis_Poser,
    DoubleRobot,
    DoubleRObot_MultiAxisPoser
};


class RobotExport MechanicGroup : public App::GeoFeature
{
    PROPERTY_HEADER(Robot::MechanicGroup);

public:
    /// Constructor
    MechanicGroup(void);
    virtual ~MechanicGroup();

    // Document Obejct Operations
    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderMechanicGroup";
    }
    virtual App::DocumentObjectExecReturn *execute(void);
    virtual App::DocumentObjectExecReturn *recompute();
    virtual short mustExecute(void) const;
    virtual PyObject *getPyObject(void);
    virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

    // Group Setup
    const Robot6AxisObject* getLinkedRobot1Ptr() const;
    const Robot6AxisObject* getLinkedRobot2Ptr() const;
    const std::vector<MechanicDevice*>& getLinkedPositioner() const;
    bool appendRobotToGroup(const std::string& RobotName);
    void updateLinkedPositionerList(const std::vector<string> t_Names);
    void updateRobotAxisValues();
    void updateExternalAxisValue();

    // Kinematic Interfaces
    bool setupJointChain(const std::vector<Base::Placement>& poseVec);
    bool setJointAngle(int t_jntID, float jntAngle);
    bool setJointAngles(const double angles[], std::size_t size);
    const std::vector<double> getGroupJointSpeed() const;
    const std::vector<double> getExtAxisSpeedLimits() const;
    void updateAxisValues();

    bool setGroupPose(const MechPose& t_GroupPose);
    const MechPose getCurrentGroupPose(const CordType &t_Type) const;
    const std::vector<double> getCurrentExtAxisValue() const;
    bool setTipPose(const Base::Placement& tip_Pose,
                    CoordOrigin pose_Origin = CoordOrigin::World,
                    Base::Placement origin_Pose = Base::Placement());
    void setTipPoseByDraggerPose(const Base::Placement& n_DraggerPose);
    void setTipPoseByDiff(const Base::Placement& difference);

    // Tool
    bool hasTorch();
    bool hasScanner();
    const Base::Placement getToolTipTranslation() const;
    void setCurrentToolType(const ToolType &t_Type);
    void setCurrentToolActive(bool activated);
    const ToolType getCurrentTool() const;

    std::vector<App::DocumentObject*> getChildrenList() const;

    const Base::Placement getGroupTipPose(CoordOrigin ref_Origin = CoordOrigin::World,
                                          Base::Placement origin_Pose = Base::Placement()) const;

    const Base::Placement getGroupFlanPose(const CoordOrigin& ref_Origin,
                                             Base::Placement origin_Pose = Base::Placement()) const;
    const Base::Placement getTeachDraggerPose() const;
    const Base::Placement getJointTransformation(const int jntID) const;
    const Base::Placement getCurrentBasePose() const;

    // Robot Kinematics
    bool setCurrentPoseAsHome();
    bool restoreHomePose();
    const Base::Placement getActiveRobotBase() const;
    void restoreJointLimits();
    const uint getJointNumbers() const;
    const float getJointAngle(const int t_jntID) const;
    const std::vector<double> getJointAngles() const;
    const std::vector<double> getExtAxisVals() const;
    const float getJointMaxAngle(const uint t_jntID) const;
    const float getJointMinAngle(const uint t_jntID) const;
    const std::string getActiveOperatorName() const;
    void setActiveRobot(const uint t_ID);
    void changeOperator();

    // Base Placement
    void setEditingStatus(bool editing){
        flag_isEditing = editing;
    }

    bool isEditing(){
        return flag_isEditing;
    }

    bool isRestoring(){
        return flag_restoringObject;
    }

    void setTeachCoordType(const TeachCoord& t_coord){
        m_TeachCoord = t_coord;
        TeachCoordIndex.setValue(t_coord);
    }

    const TeachCoord& getCurrentTeachCoord() const {
        return m_TeachCoord;
    }

    // Communication
    void setStationAddress(const std::string t_IP, const uint t_PortNum);
    bool stationConnected();
    bool connectToRobot();
    void disconnectRobot();
    bool startUpdatePose();
    void stopUpdatePose();

public:
    // Files
    App::PropertyFileIncluded File_URDF;
    App::PropertyFileIncluded File_Mesh;

    // Kinematics
    App::PropertyPlacement Pose_GroupActiveFlan;
    App::PropertyBool      Activated;
    App::PropertyInteger   TeachCoordIndex;
    App::PropertyInteger   ActiveToolIndex;

    App::PropertyString     LinkedRobotName_1;
    App::PropertyString     LinkedRobotName_2;
    App::PropertyInteger    ActiveRobotIndex;
    App::PropertyStringList LinkedPoserNames;

    // Kinematic
    App::PropertyBool       JoinMovement;
    App::PropertyFloatList  AxisValues;
    App::PropertyFloatList  GroupHomePose;
    App::PropertyBool       NetworkConnected;

    // Switch
    App::PropertyBool InteractiveTeach;
    App::PropertyBool Visiable;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    void onDocumentRestored() override;
    void updateAxisProperties();

protected:
    KinematicModel m_kinematicModel;

private:
    bool flag_updateJntVals = false;
    bool flag_updateTcpPose = false;
    bool flag_updateTCPAngle = false;
    bool flag_isEditing = false;
    bool flag_torchAssembled = false;
    bool flag_sensorAssembled = false;
    bool flag_restoringObject = false;
    CoordOrigin operatingCoord;
    TeachCoord m_TeachCoord = TeachCoord::RobotBase;
    Robot6AxisObject* m_ActiveRobotPtr = nullptr;
    Robot6AxisObject* m_LinkedRobot_1 = nullptr;
    Robot6AxisObject* m_LinkedRobot_2 = nullptr;
    std::vector<MechanicDevice*> m_ExternalAxisList;
    std::shared_ptr<ControllerTcpConnector> m_CommManager = nullptr;
    std::string m_ControllerIP;
    uint m_ControllerPort;
};

}


#endif
