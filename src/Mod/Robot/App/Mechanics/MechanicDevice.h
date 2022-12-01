// Created by Yixiao 2022-08-08

#ifndef ROBOT_POSITIONER_H
#define ROBOT_POSITIONER_H

#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include <App/PropertyLinks.h>
//#include

#include "KinematicModel.h"
#include "Mod/Robot/App/Utilites/FrameObject.h"
#include "Mod/Robot/App/PlanningObj/PlanningObject.h"
#include "Mod/Robot/App/Mechanics/Robot6AxisObject.h"

namespace Robot
{

class RobotExport MechanicDevice : public App::GeoFeature
{
    PROPERTY_HEADER(Robot::MechanicDevice);

public:
    /// Constructor
    MechanicDevice(void);
    virtual ~MechanicDevice();

    // Document Obejct Operations
    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderMechanicDevice";
    }
    virtual App::DocumentObjectExecReturn *execute(void);
    virtual App::DocumentObjectExecReturn *recompute();
    virtual short mustExecute(void) const;
    virtual PyObject *getPyObject(void);
    virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

    // Robot Kinematic Interfaces
    const std::vector<std::string> getAxisNames() const;
    bool setupJointChain(const std::vector<Base::Placement>& poseVec);
    KinematicModel& getKinematicModelRef();
    void setAxisOriginPose(const uint axisID, const Base::Placement& t_Pose);
    const Base::Placement getAxisOriginPose(const uint t_AxisID);
    const Base::Placement getOriginPose() const;

    // Joint Value setter
    bool setJointAngle(int jntID, float jntAngle);
    bool setJointAngles(const std::vector<double>& t_angles);
    void updateAxisValues();
    const std::vector<double> getAxisRuningSpeed() const;

    // TODO: Add Tool Function Back
    bool setTipPose(const Base::Placement& tip_Pose,
                    CoordOrigin pose_Origin = CoordOrigin::World,
                    Base::Placement origin_Pose = Base::Placement());
    void setTipPoseByDraggerPose(const Base::Placement& n_DraggerPose);


    // Mechanic Device Load
    void loadRobot(const char* robotName);
    void unloadRobot(const char* robotName);
    void loadWorkPiece(const char* obejctName);
    void unloadWorkPiece();

    void udpateLoadPosition();

    std::vector<App::DocumentObject*> getChildrenList() const;
    const Base::Placement getToolTipTranslation() const;
    const Base::Placement getCurrentTipPose(CoordOrigin ref_Origin = CoordOrigin::World,
                                            Base::Placement origin_Pose = Base::Placement()) const;
    const Base::Placement getCurrentFlanPose(const CoordOrigin& ref_Origin,
                                             Base::Placement origin_Pose = Base::Placement()) const;
    const Base::Placement getTeachDraggerPose() const;

    const Base::Placement getJointTransformation(const int jntID) const;

    // Mechanic Configuration
    bool isAxisDirInverted(uint jntID);
    bool flipAxisDirection(uint jntID, bool invert);
    bool restoreHomePose();
    bool setCurrentPoseAsHome();
    void restoreJointLimits();
    const int getJointNumbers() const;
    const float getJointAngle(const int JointID) const;
    const std::vector<double> getJointAngles() const;
    const float getJointMaxAngle(const uint JointID) const;
    const float getJointMinAngle(const uint JointID) const;
    const std::vector<double> getJointMaxAngles() const;
    const std::vector<double> getJointMinAngles() const;

    // Calibration
    bool calibrateAxisPose(const std::vector<Base::Placement> t_Poses, const uint t_JntID);

    // Base Placement
    void setAttachable(bool attachable);
    void setBaseToSelectedFaceCenter();
    void setTeachCoordType(const TeachCoord& t_coord){
        m_TeachCoord = t_coord;
        TeachCoordIndex.setValue(t_coord);
    }
    const TeachCoord& getCurrentTeachCoord() const {
        return m_TeachCoord;
    }

public:
    // Files
    App::PropertyFileIncluded File_URDF;
    App::PropertyFileIncluded File_Mesh;
    App::PropertyString       Path_CalibFile;
    // Obj Property
    App::PropertyString OriginReference;
    App::PropertyPlacement Pose_Reference;  // TODO: Use link property
    App::PropertyPlacement Pose_Ref2Base;
    App::PropertyBool Activated;
    App::PropertyBool isDriven;
    App::PropertyBool UseTracIK;
    App::PropertyInteger TeachCoordIndex;
    App::PropertyBool Editing;

    // IK Trigger
    App::PropertyPlacement Pose_Tip;

    // Tool
    App::PropertyString MountedWorkPieceName;
    App::PropertyString MountedRobotName;

    // Linked Feature
    App::PropertyLinkSub   LinkedFaceFeature;
    App::PropertyLinkSub   LinkedEdgeFeature;

    // Kinematic
    App::PropertyInteger   DeviceType;
    App::PropertyFloatList AxisValues;
    App::PropertyFloatList AxisRatedSpeed;
    App::PropertyFloat     AxisSpeedRatio;
    App::PropertyFloatList UpperLimits_Real;
    App::PropertyFloatList UpperLimits_Soft;
    App::PropertyFloatList LowerLimits_Real;
    App::PropertyFloatList LowerLimits_Soft;

    // Configuration
    App::PropertyFloatList HomePose;

    // Switch
    App::PropertyBool MoveWithAttachedBody;
    App::PropertyBool InteractiveTeach;
    App::PropertyBool Visiable;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);

    void onDocumentRestored() override;

protected:
    KinematicModel m_kinematicModel;

private:
    bool flag_updateJntVals = false;
    bool flag_updateTcpPose = false;
    bool flag_updateTCPAngle = false;

    CoordOrigin operatingCoord;
    Robot::TeachCoord m_TeachCoord = TeachCoord::RobotBase;
    Robot::PlanningObject* m_MountedObject = nullptr;
    Robot::Robot6AxisObject* m_MountedRobotPtr = nullptr;
    std::vector<Base::Placement> m_AxisOrigins;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H
