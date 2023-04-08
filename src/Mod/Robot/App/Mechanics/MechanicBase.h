// Created by Yixiao 2023-03-10

#ifndef ROBOT_MECHBASE_H
#define ROBOT_MECHBASE_H

#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include <App/PropertyLinks.h>
//#include
#include "KinematicModel.h"

namespace Robot
{

// This is the base class for all mechanic devices
class RobotExport MechanicBase : public App::GeoFeature
{
    PROPERTY_HEADER(Robot::MechanicBase);

public:
    /// Constructor
    MechanicBase(void);
    virtual ~MechanicBase();

    // Document Obejct Operations
    /// returns the type name of the ViewProvider
    virtual PyObject *getPyObject(void) override;
    virtual const char* getViewProviderName(void) const override{
        return "RobotGui::ViewProviderMechanicBase";
    };
    virtual App::DocumentObjectExecReturn *execute(void) override;
    virtual App::DocumentObjectExecReturn *recompute() override;
    virtual short mustExecute(void) const override;
    virtual void Save (Base::Writer &/*writer*/) const override;
    virtual void Restore(Base::XMLReader &/*reader*/) override;

    // Robot Kinematic Interfaces
    const std::vector<std::string> getAxisNames() const;
    bool setupJointChain(const std::vector<Base::Placement>& poseVec);
    KinematicModel& getKinematicModelRef();
    void setAxisOriginPose(const uint axisID, const Base::Placement& t_Pose);
    const Base::Placement getAxisOriginPose(const uint t_AxisID);
    const Base::Placement getOriginPose() const;
    // TODO: Add Tool Function Back
    const Base::Placement getCurrentTipPose() const;
    const Base::Placement getJointTransformation(const int jntID) const;

    // Joint/Axis operations
    bool setAxisHomePose();
    bool resAxisHomePose();
    // Joint Value setter
    virtual bool setJointAngle(const size_t jntID, float jntAngle);
    bool setJointAngles(const std::vector<double>& t_angles);
    void updateAxisValues();
    const std::vector<double> getAxisRuningSpeed() const;

    bool isAxisDirInverted(uint jntID);
    bool flipAxisDirection(uint jntID, bool invert);
    void updateJointLimits();
    const std::vector<double> getJointAngles() const;
    const std::vector<double> getJointMaxAngles() const;
    const std::vector<double> getJointMinAngles() const;
    float getJointAngle(const int JointID) const;
    size_t getJointNumbers() const;
    float getJointMaxAngle(const uint JointID) const;
    float getJointMinAngle(const uint JointID) const;

public:
    // Files
    App::PropertyString FilePath_URDF;
    App::PropertyString FilePath_Calibration;

    // IK Trigger
    App::PropertyPlacement Pose_Reference;  // TODO: Use link property
    App::PropertyPlacement Trans_Ref2Base;
    App::PropertyPlacement Pose_Tip;

    App::PropertyBool Visible;
    App::PropertyBool Activated;
    App::PropertyBool isDriven;
    App::PropertyBool isEditing;
    App::PropertyBool UseTracIK;

    // Linked Feature
    App::PropertyLinkSub   LinkedFaceFeature;
    App::PropertyLinkSub   LinkedEdgeFeature;

    // Kinematic
    App::PropertyFloatList AxisValues;
    App::PropertyFloatList HomePose;
    App::PropertyFloatList AxisRatedSpeed;
    App::PropertyFloat     AxisSpeedRatio;

    App::PropertyFloatList UpperLimits_Real;
    App::PropertyFloatList UpperLimits_Soft;
    App::PropertyFloatList LowerLimits_Real;
    App::PropertyFloatList LowerLimits_Soft;

    // Configuration
    App::PropertyInteger   DeviceType;

    // Switch
    App::PropertyBool Visiable;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    void onDocumentRestored() override;

protected:
    bool flag_updateJntVals = false;
    bool flag_updateTcpPose = false;
    bool flag_updateTCPAngle = false;
    KinematicModel m_kinematicModel;
    std::vector<Base::Placement> m_AxisOrigins;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H
