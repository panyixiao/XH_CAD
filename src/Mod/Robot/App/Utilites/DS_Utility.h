#ifndef DS_UTILITY_H
#define DS_UTILITY_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Base/Matrix.h>
#include <Base/Placement.h>
#include <Mod/Part/App/PartFeature.h>
#include <Inventor/nodes/SoTransform.h>
#include "Mod/Robot/App/kdl_cp/chain.hpp"
#include "Mod/Robot/App/kdl_cp/jntarray.hpp"

#include "FrameObject.h"

using namespace std;
using namespace Part;
namespace Robot {
const std::string tmpMeshFilePath = "/tmp/";

using Cordinate = std::pair<CordType, uint>;
using CompPose = std::pair<Base::Placement, std::pair<double, double>>;

struct RobotPose
{
    RobotPose() {
        CordInfo = std::make_pair(CordType::WCS,0);
    }
    void setPoseData(const Base::Placement& t_Pose);
    void setPoseData(const CompPose& t_Pose);
    void setPoseData(const std::vector<double>& t_JntVals);
    bool isValid() const{
        if(CordInfo.first == CordType::WCS){
            return FlanPose != Base::Placement();
        }
        return true;
    }
    void Save (Base::Writer &/*writer*/) const;
    void Restore(Base::XMLReader &/*reader*/);

    const Base::Placement getCartPose() const;

    Cordinate CordInfo;
    uint ConfigID = 1;
    std::vector<double> PoseData = std::vector<double>(8,0.0);  // tX,tY,tZ,rZ,rY,rX
    Base::Placement FlanPose;
};

struct GroupPose
{
public:
    RobotPose Pose_Rbt1;
    RobotPose Pose_Rbt2;
    std::vector<double> ExtVals = std::vector<double>(8,0.0);
    bool dynamicTrac = false;
public:
    GroupPose(){}
    bool isValid() const{
        return Pose_Rbt1.isValid() || Pose_Rbt2.isValid();
    }
    void setExtVals(const KDL::JntArray& t_JntVals);
    void Save (Base::Writer &/*writer*/) const;
    void Restore(Base::XMLReader &/*reader*/);
};

class DS_Utility {
public:
  DS_Utility();
  static float constexpr s_ros2cad_lengthFactor = 1000.0;

  static Eigen::Matrix3d vectorProduct(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
  static double dotProduct(const Base::Vector3d& v1, const Base::Vector3d& v2);
  static Base::Vector3d crossProduct(const Base::Vector3d& v1, const Base::Vector3d& v2);

  static std::string double2string(const double val, const double precision);

  static bool ifVector3dSame(Base::Vector3d const &vec1,
                             Base::Vector3d const &vec2, float epsilon = 10e-5);
  static bool ifRotationSame(Base::Rotation const &r1, Base::Rotation const &r2,
                             float epsilon = 10e-5);
  static bool ifPlacementSame(Base::Placement const &pose1,
                              Base::Placement const &pose2,
                              float epsilon = 10e-5);
  static float getPoseCartDist(const Base::Placement& src_Pose,
                               const Base::Placement& dst_Pose);
  /// return true if (different and set value)
  /// return false if (same, and not set value)
  static bool setValueIfDifferent(App::PropertyPlacement &placement,
                                  Base::Placement const &newPose);
  /// return true if (different and set value)
  /// return false if (same, and not set value)

  static void convert_Placement2Transform(const Base::Placement &from,
                                          SoTransform *to);

  static void generateTmpSTLFile(const PropertyPartShape &targetShape,
                                 const string &objectName);

  static Base::Placement calculateAveragePose(Base::Placement const &pose_1,
                                              Base::Placement const &pose_2);
  static Base::Rotation convertFromEulerAngle(const double roll,
                                              const double pitch,
                                              const double yaw);

  static Base::Placement calculateDiffPlacement(const Base::Placement &src,
                                                const Base::Placement &des);

  static Base::Placement translateByPlacement(const Base::Placement &src,
                                              const Base::Placement &trans);

  static Base::Placement getInverseTrans(const Base::Placement &trans);

  static void PlacementToTransform(SoTransform *&targetTrans,
                                   const Base::Placement &trans);

  static void split(std::string const& string, const char delemiter, std::vector<std::string>& destination);

  static double convert_Rad2Deg(double inputRad);
  static double convert_Deg2Rad(double inputDeg);

  // Calibration
  static const Base::Placement calculateTargetCalibArcCenterPose(const std::vector<Base::Placement> t_calibData);
  static const Base::Placement calculateArcCenterPose(const Base::Placement& p1,
                                                      const Base::Placement& p2,
                                                      const Base::Placement& p3);

};
}

#endif // DS_UTILITY_H
