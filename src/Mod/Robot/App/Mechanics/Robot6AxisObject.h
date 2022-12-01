// Created by Yixiao 2022-04-16


#ifndef ROBOT_ROBOT6AXISOBJECT_H
#define ROBOT_ROBOT6AXISOBJECT_H

#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include <App/PropertyLinks.h>
//#include

#include "KinematicModel.h"
#include "Mod/Robot/App/Trac/RobotWaypoint.h"
#include "Mod/Robot/App/Utilites/FrameObject.h"
#include "Mod/Robot/App/Tool/ScannerObject.h"
#include "Mod/Robot/App/Tool/TorchObject.h"

namespace Robot
{
class RobotExport Robot6AxisObject : public App::GeoFeature
{
    PROPERTY_HEADER(Robot::Robot6AxisObject);

public:
    /// Constructor
    Robot6AxisObject(void);
    virtual ~Robot6AxisObject();

    // Document Obejct Operations
    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderRobot6AxisObject";
    }
    virtual App::DocumentObjectExecReturn *execute(void);
    virtual App::DocumentObjectExecReturn *recompute();
    virtual short mustExecute(void) const;
    virtual PyObject *getPyObject(void);
    virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

    // Robot Kinematic Interfaces
    bool setupJointChain(const std::vector<Base::Placement>& poseVec);
    KinematicModel& getKinematicModelRef();
    // Axis Value setter
    bool setJointAngle(int jntID, float jntAngle);
    bool setAxisValues(const std::vector<double> t_Vals);
    void updateAxisValues();
    const std::vector<double> getAxisRunningSpeeds() const;
    // Pose Value
    bool setRobotPose(const RobotPose &t_Pose);
    const RobotPose getRobotPose(const CordType& t_Type) const;
    bool setRobotTipPose(const Base::Placement& tip_Pose,
                         CoordOrigin pose_Origin = CoordOrigin::World,
                         Base::Placement origin_Pose = Base::Placement());
    void setTipPoseByDragger(const Base::Placement& t_DraggerPose);
    void setTipPoseByDiff(const Base::Placement& n_Stp);
    const Base::Placement getToolTipTranslation() const;
    const Base::Placement getToolFrameTrans(const uint ToolID) const;

    // Tool Setting
    void installTool(const char* tool_Name);
    void uninstallTool(Robot::ToolObject* t_Tool);
    void udpateToolPosition();
    void setCurrentToolType(const ToolType& t_Type);
    void setCurrentToolActive(bool activated = false);
    const ToolType getCurrentTool() const;

    bool hasTorch(){
        return m_Torch != nullptr;
    }
    bool hasScanner(){
        return m_Scanner != nullptr;
    }

    std::vector<App::DocumentObject*> getChildrenList() const;

    const Base::Placement getCurrentTipPose(CoordOrigin ref_Origin = CoordOrigin::World,
                                            Base::Placement origin_Pose = Base::Placement()) const;
    const Base::Placement getCurrentFlanPose(const CoordOrigin& ref_Origin,
                                             Base::Placement origin_Pose = Base::Placement()) const;
    const Base::Placement getTeachDraggerPose() const;
    const Base::Placement getJointTransformation(const int jntID) const;
    const Base::Placement getCurrentSensorTipPose(const CoordOrigin &coordType) const;
    const Base::Placement getCurrentBasePose() const;
    const Base::Placement getSelectedFeatureCenter() const;

    // Robot Config
    bool restoreHomePose();
    bool setCurrentPoseAsHome();
    void restoreJointLimits();
    bool JointDirInverted(uint j_id);
    bool flipAxisDirection(uint jntID, bool invert);
    uint getJointNumbers() const;
    const float getJointAngle(const int t_ID) const;
    const std::vector<double> getJointAngles() const;
    const float getJointMaxAngle(const int t_ID) const;
    const float getJointMinAngle(const int t_ID) const;
    const std::vector<double> getJointMaxAngles() const;
    const std::vector<double> getJointMinAngles() const;

    // Base Placement
    void setAttachable(bool attachable);
    void setBaseToSelectedFaceCenter();
    void setEditingStatus(bool editing){
        flag_isEditing = editing;
    }

    void setTorchAssembled(bool assemebled){
        flag_torchAssembled = assemebled;
    }
    void setSensorAssembled(bool assemebled){
        flag_sensorAssembled = assemebled;
    }
    bool isEditing(){
        return flag_isEditing;
    }
    bool isTorchAssembled() const {
        return flag_torchAssembled;
    }
    bool isSenorAssembled() const{
        return flag_sensorAssembled;
    }

    void setTeachCoordType(const TeachCoord& t_coord){
        m_TeachCoord = t_coord;
        TeachCoordIndex.setValue(t_coord);
    }
    const TeachCoord& getCurrentTeachCoord() const {
        return m_TeachCoord;
    }

    const bool ScannerAssembled() const{
        return m_Scanner != nullptr;
    }
    const bool TorchAssembled() const{
        return m_Torch != nullptr;
    }

public:
    // Files
    App::PropertyString File_URDF;

    // Obj Property
    App::PropertyPlacement Pose_Ref2Base;
    App::PropertyPlacement Pose_Reference;
    App::PropertyString    ExternalAxisName;
    App::PropertyBool      Activated;
    App::PropertyBool      isDriven;
    App::PropertyBool      UseTracIK;
    App::PropertyBool      EnableArmConfiguration;
    App::PropertyInteger   TeachCoordIndex;

    // IK Trigger
    App::PropertyPlacement Pose_Flan;

    // Tool
    App::PropertyInteger   CurrentToolIndex;
    App::PropertyInteger   TorchIndex;
    App::PropertyInteger   ScannerIndex;
    App::PropertyInteger   CameraIndex;

    // Torch
    App::PropertyLink      TorchShape;
    App::PropertyPlacement Trans_Flan2TorchBase;  // By changing this value to adjust model attachment, make simulation more realistic
    App::PropertyPlacement Trans_Flan2TorchTip; // Based on Calibrated Result

    // Sensor
    App::PropertyLink      SensorShape;
    App::PropertyPlacement Trans_Flan2SensorBase;
    App::PropertyPlacement Trans_Flan2SensorOrigin; // Based on Calibrated Result
    App::PropertyPlacement Trans_SensorOrigin2Tip;

    App::PropertyString    ScannerName;
    App::PropertyString    TorchName;

    // Linked Feature
    App::PropertyLinkSub   LinkedFaceFeature;
    App::PropertyLinkSub   LinkedEdgeFeature;

    // Kinematic
    App::PropertyFloatList MainAxisValues;
    App::PropertyFloatList ExtrAxisValues;
    App::PropertyFloatList AxisRatedSpeed;
    App::PropertyFloat     AxisSpeedRatio;
    App::PropertyFloatList UpperLimits_Real;
    App::PropertyFloatList UpperLimits_Soft;
    App::PropertyFloatList LowerLimits_Real;
    App::PropertyFloatList LowerLimits_Soft;

    // Configuration
    App::PropertyFloatList HomePose;
    App::PropertyIntegerList ArmConfiguration;  // Wrist | Forearm | Elbow

    App::PropertyBool Wrist_NonFlip;
    App::PropertyBool ForeArm_onRight;
    App::PropertyBool Elbow_Upward;

    // Switch
    App::PropertyBool MoveWithAttachedBody;
    App::PropertyBool InteractiveTeach;
    App::PropertyBool Visible;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    void onDocumentRestored() override;
    void updateRobotConfiguration();

protected:
    KinematicModel m_kinematicModel;

private:
    bool flag_updateJntVals = false;
    bool flag_updateTcpPose = false;
    bool flag_updateTCPAngle = false;
    bool flag_isEditing = false;
    bool flag_torchAssembled = false;
    bool flag_sensorAssembled = false;
    CoordOrigin operatingCoord;
    TeachCoord m_TeachCoord = TeachCoord::RobotBase;

    Robot::ScannerObject* m_Scanner = nullptr;
    Robot::TorchObject* m_Torch = nullptr;
    std::vector<Base::Placement> m_ToolTranslateVec;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H
