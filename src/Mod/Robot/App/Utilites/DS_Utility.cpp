#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "eigen3/Eigen/StdVector"
#include <Mod/Mesh/App/Mesh.h>
#include <Mod/MeshPart/App/Mesher.h>
#include <StlAPI.hxx>
//#include <oce/TopLoc_Location.hxx>
#include <gp_Trsf.hxx>

//#include <g3log/rlog.h>

#include "DS_Utility.h"
using namespace Robot;
using namespace std;

DS_Utility::DS_Utility() {}

Eigen::Matrix3d DS_Utility::vectorProduct(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
    Eigen::Matrix3d result;

    result<<v1[0]*v2[0], v1[0]*v2[1], v1[0]*v2[2],
            v1[1]*v2[0], v1[1]*v2[1], v1[1]*v2[2],
            v1[2]*v2[0], v1[2]*v2[1], v1[2]*v2[2];
    return result;
}

double DS_Utility::dotProduct(const Base::Vector3d &v1, const Base::Vector3d &v2)
{
    double result;
    result = (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
    return result;
}

Base::Vector3d DS_Utility::crossProduct(const Base::Vector3d &v1, const Base::Vector3d &v2)
{
    Base::Vector3d result;

    result.x = v1.y*v2.z - v2.y*v1.z;
    result.y = v2.x*v1.z - v2.z*v1.x;
    result.z = v1.x*v2.y - v2.x*v1.y;

    return result;
}

string DS_Utility::double2string(const double val, const double precision)
{
    std::stringstream ss;
    ss<<std::setiosflags(std::ios::fixed)<<std::setprecision(precision)<<val;
    return ss.str();
}

//boost::shared_ptr<shapes::Mesh>
//DS_Utility::generate_rosShapeMesh_from_partShape(const PropertyPartShape &targetShape, double accuracy) {
//  std::vector<Base::Vector3d> vertices;
//  std::vector<Data::ComplexGeoData::Facet> facets;
//  auto data = targetShape.getComplexData();
//  data->getFaces(vertices, facets, accuracy);
//  /// Method 1
//  EigenSTL::vector_Vector3d e_vertices;
//  for (auto const &pnt : vertices) {
//    Eigen::Vector3d e_pnt(pnt.x / s_ros2cad_lengthFactor,
//                          pnt.y / s_ros2cad_lengthFactor,
//                          pnt.z / s_ros2cad_lengthFactor);
//    e_vertices.push_back(e_pnt);
//  }
//  vector<unsigned int> index_num;
//  for (auto const &face : facets) {
//    index_num.push_back(face.I1);
//    index_num.push_back(face.I2);
//    index_num.push_back(face.I3);
//  }
//  auto mesh_ptr = shapes::createMeshFromVertices(e_vertices);
//  if (mesh_ptr != nullptr) {
//    return boost::make_shared<shapes::Mesh>(*mesh_ptr);
//  }
//  return nullptr;

//  //  /// Method 2
//  //  auto ros_mesh =  boost::make_shared<shapes::Mesh>(vertices.size(),
//  //  facets.size());
//  //  int count_i = 0;
//  //  for (auto const &pnt : vertices) {
//  //    ros_mesh->vertices[count_i] = pnt.x / s_ros2cad_lengthFactor;
//  //    ros_mesh->vertices[count_i + 1] = pnt.y / s_ros2cad_lengthFactor;
//  //    ros_mesh->vertices[count_i + 2] = pnt.z / s_ros2cad_lengthFactor;
//  //    count_i += 3;
//  //  }
//  //  count_i = 0;
//  //  for (auto const& face:facets) {
//  //    auto p1 = (int)face.I1;
//  //    auto p2 = (int)face.I2;
//  //    auto p3 = (int)face.I3;
//  //    ros_mesh->triangles[count_i] = p1;
//  //    ros_mesh->triangles[count_i + 1] = p2;
//  //    ros_mesh->triangles[count_i + 2] = p3;
//  //    count_i += 3;
//  //  }
//  //  return ros_mesh;
//}

//geometry_msgs::PoseStamped
//DS_Utility::PlacementToPoseStamped(const Base::Placement &base_Pose,
//                                   std::string frameID, float unit_scale) {
//  geometry_msgs::PoseStamped newPose;
//  newPose.header.frame_id = frameID;
//  // Transform from Milimeter -> meter
//  newPose.pose.position.x = base_Pose.getPosition().x / unit_scale;
//  newPose.pose.position.y = base_Pose.getPosition().y / unit_scale;
//  newPose.pose.position.z = base_Pose.getPosition().z / unit_scale;
//  double q0, q1, q2, q3;
//  base_Pose.getRotation().getValue(q0, q1, q2, q3);
//  newPose.pose.orientation.w = q3;
//  newPose.pose.orientation.x = q0;
//  newPose.pose.orientation.y = q1;
//  newPose.pose.orientation.z = q2;
//  return newPose;
//}

bool DS_Utility::ifVector3dSame(Base::Vector3d const &vec1,
                                Base::Vector3d const &vec2, float epsilon) {
  bool retSame = true;

  if (std::abs(vec1.x - vec2.x) > epsilon)
    return !retSame;
  if (std::abs(vec1.y - vec2.y) > epsilon)
    return !retSame;
  if (std::abs(vec1.z - vec2.z) > epsilon)
    return !retSame;

  return retSame;
}

bool DS_Utility::ifRotationSame(Base::Rotation const &r1,
                                Base::Rotation const &r2, float epsilon) {
  bool retSame = true;
  if (std::abs(r1.getValue()[0] - r2.getValue()[0]) > epsilon)
    return !retSame;
  if (std::abs(r1.getValue()[1] - r2.getValue()[1]) > epsilon)
    return !retSame;
  if (std::abs(r1.getValue()[2] - r2.getValue()[2]) > epsilon)
    return !retSame;
  if (std::abs(r1.getValue()[3] - r2.getValue()[3]) > epsilon)
    return !retSame;
  return retSame;
}

bool DS_Utility::ifPlacementSame(Base::Placement const &pose1,
                                 Base::Placement const &pose2,
                                 float epsilon) {
  bool retSame = true;
  if (!ifVector3dSame(pose1.getPosition(), pose2.getPosition(), epsilon))
    return !retSame;
  if (!ifRotationSame(pose1.getRotation(), pose2.getRotation(), epsilon))
    return !retSame;

  return retSame;
}

float DS_Utility::getPoseCartDist(const Base::Placement &src_Pose, const Base::Placement &dst_Pose)
{
    auto diffPose = src_Pose.inverse() * dst_Pose;
    return diffPose.getPosition().Length();
}

bool DS_Utility::setValueIfDifferent(App::PropertyPlacement &placement,
                                     Base::Placement const &newPose) {
  if (!ifPlacementSame(placement.getValue(), newPose)) {
    placement.setValue(newPose);
    return true;
  }

  return false;
}

//bool DS_Utility::setValueIfDifferent(App::PropertyPlacement &placement,
//                                     geometry_msgs::Pose const &newPose) {
//  return setValueIfDifferent(placement, PoseToPlacement(newPose));
//}

//Base::Placement
//DS_Utility::PoseToPlacement(const geometry_msgs::Pose &base_Pose,
//                            float unit_scale) {
//  Base::Placement poseRet;
//  Base::Vector3d vec;
//  vec.x = base_Pose.position.x * DS_Utility::s_ros2cad_lengthFactor;
//  vec.y = base_Pose.position.y * DS_Utility::s_ros2cad_lengthFactor;
//  vec.z = base_Pose.position.z * DS_Utility::s_ros2cad_lengthFactor;
//  poseRet.setPosition(vec);

//  Base::Rotation orientation;
//  double quat[4];
//  quat[3] = base_Pose.orientation.w;
//  quat[0] = base_Pose.orientation.x;
//  quat[1] = base_Pose.orientation.y;
//  quat[2] = base_Pose.orientation.z;
//  orientation.setValue(quat);
//  poseRet.setRotation(orientation);
//  return poseRet;
//}

//Base::Placement
//DS_Utility::Transform2Placement(const geometry_msgs::Transform &trans,
//                                float unit_scale) {
//  Base::Placement poseRet;
//  Base::Vector3d vec;
//  vec.x = trans.translation.x * DS_Utility::s_ros2cad_lengthFactor;
//  vec.y = trans.translation.y * DS_Utility::s_ros2cad_lengthFactor;
//  vec.z = trans.translation.z * DS_Utility::s_ros2cad_lengthFactor;
//  poseRet.setPosition(vec);
//  Base::Rotation orientation;
//  double quat[4];
//  quat[3] = trans.rotation.w;
//  quat[0] = trans.rotation.x;
//  quat[1] = trans.rotation.y;
//  quat[2] = trans.rotation.z;
//  orientation.setValue(quat);
//  poseRet.setRotation(orientation);
//  return poseRet;
//}

void DS_Utility::generateTmpSTLFile(const PropertyPartShape &targetShape,
                                    const string &objectName) {
  string filePath = tmpMeshFilePath + objectName + string(".stl");
  TopoDS_Shape tmpShap = targetShape.getShape().getShape();
  //  auto testLoc = targetShape.getShape()._Shape.Location().Transformation();
  TopLoc_Location tmpLoc;
  tmpLoc.Identity();
  tmpShap.Location(tmpLoc);
  StlAPI::Write(tmpShap, filePath.c_str(), false);
}

//std::shared_ptr<shapes::Mesh> DS_Utility::retrieveTmpSTLFile(const string &objectName,
//                                                  double scale) {
//  string filePath =
//      string("file://") + tmpMeshFilePath + objectName + string(".stl");
//  Eigen::Vector3d mesh_scale(scale, scale, scale);
//  auto meshPtr = shapes::createMeshFromResource(filePath, mesh_scale);
//  if (meshPtr != nullptr) {
//    return std::make_shared<shapes::Mesh>(*meshPtr);
//  }
//  return nullptr;
//}

void DS_Utility::convert_Placement2Transform(const Base::Placement &from,
                                             SoTransform *to) {
  float q0 = (float)from.getRotation().getValue()[0];
  float q1 = (float)from.getRotation().getValue()[1];
  float q2 = (float)from.getRotation().getValue()[2];
  float q3 = (float)from.getRotation().getValue()[3];
  float px = (float)from.getPosition().x;
  float py = (float)from.getPosition().y;
  float pz = (float)from.getPosition().z;
  to->rotation.setValue(q0, q1, q2, q3);
  to->translation.setValue(px, py, pz);
  to->center.setValue(0.0f, 0.0f, 0.0f);
  to->scaleFactor.setValue(1.0f, 1.0f, 1.0f);
}

Base::Placement DS_Utility::calculateAveragePose(Base::Placement const &pose_1,
                                                 Base::Placement const &pose_2) {
    auto avr_Trans = 0.5*(pose_1.getPosition() + pose_2.getPosition());
    // Todo: Adding Rotation average

    return Base::Placement(avr_Trans, Base::Rotation());
}

Base::Rotation DS_Utility::convertFromEulerAngle(const double roll,
                                                 const double pitch,
                                                 const double yaw) {
  auto result = Base::Rotation();
  result.setYawPitchRoll(yaw, pitch, roll);
  return result;
}

Base::Placement DS_Utility::calculateDiffPlacement(const Base::Placement &src,
                                                   const Base::Placement &dst) {
  auto srcMat = src.toMatrix();
  srcMat.inverse();
  return Base::Placement(srcMat * dst.toMatrix());
}

Base::Placement DS_Utility::translateByPlacement(const Base::Placement &src,
                                                 const Base::Placement &trans) {
  return Base::Placement(src.toMatrix() * trans.toMatrix());
}

Base::Placement DS_Utility::getInverseTrans(const Base::Placement &trans) {
  auto originalTransMat = trans.toMatrix();
  originalTransMat.inverse();
  return Base::Placement(originalTransMat);
}

void DS_Utility::PlacementToTransform(SoTransform *&targetTrans,
                                      const Base::Placement &trans) {

  if (targetTrans == nullptr)
    return;
  targetTrans->translation.setValue(
      trans.getPosition().x, trans.getPosition().y, trans.getPosition().z);
  double quat[4] = {0};
  trans.getRotation().getValue(quat[0], quat[1], quat[2], quat[3]);
  targetTrans->rotation.setValue(quat[0], quat[1], quat[2], quat[3]);
}

void DS_Utility::split(std::string const& string, const char delemiter, std::vector<std::string>& destination)
{
    std::string::size_type  last_position(0);
    std::string::size_type  position(0);

    for (std::string::const_iterator it(string.begin()); it != string.end(); ++it, ++position)
    {
        if (*it == delemiter )
        {
            destination.push_back(string.substr(last_position, position - last_position ));
            last_position = position + 1;
        }
    }
    destination.push_back(string.substr(last_position, position - last_position ));
}

double DS_Utility::convert_Rad2Deg(double inputRad) {
  return inputRad * 180 / M_PI;
}

double DS_Utility::convert_Deg2Rad(double inputDeg) {
    return inputDeg * M_PI / 180;
}

const Base::Placement DS_Utility::calculateTargetCalibArcCenterPose(const std::vector<Base::Placement> t_calibData)
{
    Base::Placement t_center;
    if(t_calibData.size()%3!=0){
        return t_center;
    }

    auto totalStp = t_calibData.size()/3;
    auto stpLen = totalStp;

    for(auto i = 0; i<=totalStp; i++){
        auto _p1 = t_calibData[i];
        auto _p2 = t_calibData[i+stpLen];
        auto _p3 = t_calibData[i+2*stpLen];
        auto tmp_center = DS_Utility::calculateArcCenterPose(_p1,_p2,_p3);

        auto vec_1 = _p1.getPosition() - tmp_center.getPosition();
        auto vec_2 = _p2.getPosition() - tmp_center.getPosition();

        auto rot_Norm = crossProduct(vec_1, vec_2).Normalize();
        if(i == 0)
            t_center = tmp_center;
        else{
            if(DS_Utility::getPoseCartDist(t_center, tmp_center)<15.0)  //mm
                t_center = calculateAveragePose(t_center,tmp_center);
        }
    }
    return t_center;
}

const Base::Placement DS_Utility::calculateArcCenterPose(const Base::Placement &p1,
                                                         const Base::Placement &p2,
                                                         const Base::Placement &p3)
{
    /// https://blog.csdn.net/yanmy2012/article/details/8111600
    Base::Placement t_center;
    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
    x1 = p1.getPosition().x;
    x2 = p2.getPosition().x;
    x3 = p3.getPosition().x;

    y1 = p1.getPosition().y;
    y2 = p2.getPosition().y;
    y3 = p3.getPosition().y;

    z1 = p1.getPosition().z;
    z2 = p2.getPosition().z;
    z3 = p3.getPosition().z;

    double A1,A2,A3,B1,B2,B3,C1,C2,C3,D1,D2,D3;
    A1 =  y1*z2 - y1*z3 - z1*y2 + z1*y3 + y2*z3 - y3*z2;
    B1 = -x1*z2 + x1*z3 + z1*x2 - z1*x3 - x2*z3 + x3*z2;
    C1 =  x1*y2 - x1*y3 - y1*x2 + y1*x3 + x2*y3 - x3*y2;
    D1 = -x1*y2*z3 + x1*y3*z2 + x2*y1*z3 - x3*y1*z2 - x2*y3*z1 + x3*y2*z1;

    A2 = 2*(x2 - x1);
    B2 = 2*(y2 - y1);
    C2 = 2*(z2 - z1);
    D2 = pow(x1,2) + pow(y1,2) + pow(z1,2) - pow(x2,2) - pow(y2,2) - pow(z2,2);

    A3 = 2*(x3 - x1);
    B3 = 2*(y3 - y1);
    C3 = 2*(z3 - z1);
    D3 = pow(x1,2) + pow(y1,2) + pow(z1,2) - pow(x3,2) - pow(y3,2) - pow(z3,2);

    double x=-(B1*C2*D3-B1*C3*D2-B2*C1*D3+B2*C3*D1+B3*C1*D2-B3*C2*D1)/(A1*B2*C3-A1*B3*C2-A2*B1*C3+A2*B3*C1+A3*B1*C2-A3*B2*C1);
    double y= (A1*C2*D3-A1*C3*D2-A2*C1*D3+A2*C3*D1+A3*C1*D2-A3*C2*D1)/(A1*B2*C3-A1*B3*C2-A2*B1*C3+A2*B3*C1+A3*B1*C2-A3*B2*C1);
    double z=-(A1*B2*D3-A1*B3*D2-A2*B1*D3+A2*B3*D1+A3*B1*D2-A3*B2*D1)/(A1*B2*C3-A1*B3*C2-A2*B1*C3+A2*B3*C1+A3*B1*C2-A3*B2*C1);

    t_center.setPosition(Base::Vector3d(x,y,z));



    return t_center;
}
