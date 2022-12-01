//Created By Yixiao 2022-8-23

#ifndef TRAJECTORY_JOINTSPACE_H
#define TRAJECTORY_JOINTSPACE_H

#include <memory>
#include "Mod/Robot/App/kdl_cp/path_jointspace.h"
#include "Mod/Robot/App/kdl_cp/trajectory.hpp"


namespace KDL
{
class Trajectory_JointSpace
{
    std::shared_ptr<Path_JointSpace> m_JPath = nullptr;
    std::shared_ptr<VelocityProfile_JointSpace> m_JVelLimit = nullptr;
    double m_TracDuration = 0.0;
    std::vector<double> m_Tvec;
    std::vector<JntArray> m_JVelVec;
    std::vector<double> m_Tvec_Buffer;
    std::vector<JntArray> m_JVelVec_Buffer;
    uint m_Dof = 0;
public:
    Trajectory_JointSpace();
    Trajectory_JointSpace(std::shared_ptr<Path_JointSpace> t_PathPtr,
                          std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr_limit);
    ~Trajectory_JointSpace(){};
    bool Add(std::shared_ptr<Path_JointSpace> t_PathPtr,
             std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr,
             double t_Duration = -1.0);
    const JntArray Pos(double t_sec) const;
    const double Duration() const;
protected:
    double calcTracMinExecutionTime(const std::shared_ptr<Path_JointSpace> t_PathPtr,
                                  const std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr);
    const double calculateSegmentDuration(const JntArray& j1,
                                          const JntArray& j2,
                                          const JntArray& j_Vels);
    bool adjustTracDuration(const std::shared_ptr<Path_JointSpace> t_PathPtr,
                            const std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr,
                            const double min_Duration,
                            const double tar_Duration);
    void appendTargetPath(const std::shared_ptr<Path_JointSpace> t_PathPtr,
                          const std::shared_ptr<VelocityProfile_JointSpace> t_vlpfPtr,
                          const double path_duration);
    const JntArray calcRequiredJntSpeed(const JntArray& src, const JntArray &dst, const double dT);
    bool isJVelArrayValid(const JntArray& t_JVels);

};

}


#endif
