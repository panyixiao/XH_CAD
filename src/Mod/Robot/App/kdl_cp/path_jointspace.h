//Created By Yixiao 2022-8-23

#ifndef KDL_MOTION_PATH_JOINTSPACE_H
#define KDL_MOTION_PATH_JOINTSPACE_H

#include "path.hpp"
#include <memory>


namespace KDL
{

// Unit: deg/sec
class VelocityProfile_JointSpace{
    JntArray t_Vels;
public:
    VelocityProfile_JointSpace(const std::vector<double>& jnt_Vels){
        t_Vels.resize(jnt_Vels.size());
        for(int i = 0; i<jnt_Vels.size(); i++){
            t_Vels.data(i) = jnt_Vels[i];
        }
    }
    const JntArray& getJntVels(){
        return t_Vels;
    }
};

class Path_JointSpace
{
    uint m_NbrDOF = 0;
    std::vector<JntArray> m_JFrames;
public:
    /// Constructor
    Path_JointSpace(void);
    virtual ~Path_JointSpace();
    Path_JointSpace(const JntArray& f1,
                    const JntArray& f2);
    Path_JointSpace(const JntArray& singleFrame);
    bool isValid() const;
    bool Add(const JntArray& n_Frame);
    bool Add(const std::shared_ptr<Path_JointSpace> t_PathPtr);
    const std::vector<JntArray>& getJFrames() const;
    const uint getPathDOF() const;
    virtual Path::IdentifierType getIdentifier() const {
        return Path::ID_JOINTSPACE;
    }

};

}


#endif
