// Created By Yixiao 2022-05-04

#ifndef ROBOT_RBTWAYPOINT_H
#define ROBOT_RBTWAYPOINT_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "Mod/Robot/App/Utilites/FrameObject.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include <Base/Persistence.h>
#include <Base/Placement.h>


namespace Robot
{

enum class PoseType {
    CART = 0,
    JNTS,
    INCR   // Increament Type, in Cart Space
};


/** The representation of a waypoint in a trajectory
 */
class RobotExport RobotWaypoint : public Base::Persistence
{
    TYPESYSTEM_HEADER();

public:

    RobotWaypoint();

    RobotWaypoint(const RobotWaypoint&);

    RobotWaypoint& operator=(const RobotWaypoint&);

    bool operator==(const RobotWaypoint&);

    /// Single Robot(Default Robot 1)
//    RobotWaypoint(const CompPose &t_RbtPose);

    RobotWaypoint(const CompPose &t_RbtPose,
                  const std::vector<double>& extVals,
                  uint t_ID = 1);
    /// Cart Pose
    // Single Robot
    RobotWaypoint(const Base::Placement &t_CartPose,
                  const std::vector<double>& extVals,
                  uint t_ID = 1);
    // Double Robot
    RobotWaypoint(const Base::Placement &gp1_CartPose,
                  const Base::Placement &gp2_CartPose,
                  const std::vector<double>& extVals);

    /// Joint Pose
    RobotWaypoint(const std::vector<double>& t_JntPose);
    // Single Robot
    RobotWaypoint(const std::vector<double>& t_JntPose,
                  const std::vector<double>& extVals,
                  const Base::Placement &displayPose,
                  uint t_ID = 1);

    /// Double Robot
    RobotWaypoint(const GroupPose& t_Pose);


    ~RobotWaypoint();

	// from base class
    virtual unsigned int getMemSize (void) const;
	virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

    const std::size_t getPointHashID() const{
        return m_PoseID;
    }

    void setWP_Name(const std::string& t_Name){
        m_PntName = t_Name;
    }

    void setGp1_CartPose(const Base::Placement& new_Pose){
        m_PoseData.Pose_Rbt1.setPoseData(new_Pose);
    }
    void setGp2_CartPose(const Base::Placement& new_Pose){
        m_PoseData.Pose_Rbt2.setPoseData(new_Pose);
    }

    void setGp1_CartPoseAdjust(const Base::Placement& t_diff){
        auto newPose = m_PoseData.Pose_Rbt1.getCartPose() * t_diff;
        m_PoseData.Pose_Rbt1.setPoseData(newPose);
    }
    void setGp2_CartPoseAdjust(const Base::Placement& t_diff){
        auto newPose = m_PoseData.Pose_Rbt2.getCartPose() * t_diff;
        m_PoseData.Pose_Rbt2.setPoseData(newPose);
    }

    const std::string& getWP_Name() const {
        return m_PntName;
    }

    const PoseType getWPType() const{
        return PoseType::CART;
    }

    const Base::Placement getWPCartPose_GP1() const{
        if(m_PoseData.Pose_Rbt1.isValid())
            return m_PoseData.Pose_Rbt1.getCartPose();
        return Base::Placement();
    }
    const Base::Placement getWPCartPose_GP2() const{
        if(m_PoseData.Pose_Rbt2.isValid())
            return m_PoseData.Pose_Rbt2.getCartPose();
        return Base::Placement();
    }
    const GroupPose& getWPPoseData() const{
        return m_PoseData;
    }

    const Base::Placement getWP_AJSTPose() const{
        return Base::Placement();
    }

    const std::vector<double>& getWP_JntPose() const{
        return m_PoseData.ExtVals;
    }

    const bool isValid() const;

protected:
    void updateWaypointName();

protected:
    size_t m_PoseID;
    std::string m_PntName;
    GroupPose m_PoseData;
};
}


#endif // ROBOT_WAYPOINT_H
