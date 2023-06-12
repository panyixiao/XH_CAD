// Created By Yixiao 2022-05-04

#ifndef ROBOT_RBTWAYPOINT_H
#define ROBOT_RBTWAYPOINT_H
#include "Mod/Robot/App/PreCompiled.h"
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
class RobotExport TargetPoint : public Base::Persistence
{
    TYPESYSTEM_HEADER();

public:
    TargetPoint();
    TargetPoint(const TargetPoint&);

    TargetPoint& operator=(const TargetPoint&);
    bool operator==(const TargetPoint&);
    /// Cart Pose
    TargetPoint(const CartPose& t_CartPose);
    /// Double Robot
    TargetPoint(const MechPose& t_Pose);

    ~TargetPoint();

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

    const std::string& getWP_Name() const {
        return m_PntName;
    }

    bool isValid() const;

protected:
    void updateWaypointName();

protected:
    size_t m_PoseID;
    std::string m_PntName;
    MechPose m_MechPose;
    CartPose m_CartPose;
};
}


#endif // ROBOT_WAYPOINT_H
