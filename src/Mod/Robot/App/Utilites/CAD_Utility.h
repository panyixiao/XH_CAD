#ifndef CAD_UTILITY_H
#define CAD_UTILITY_H

#include <App/DocumentObject.h>
#include <App/PropertyLinks.h>
#include <Base/Placement.h>
//#include <RD_DataStructure/RD_STL/RD_String.h>
//#include <RD_DataStructure/RD_STL/RD_Vector.h>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Vec.hxx>
class TopoDS_Wire;
class TopoDS_Edge;
class TopoDS_Face;

//using namespace RD;

// const Standard_Real face_ExpStp = 1;
// const Standard_Real edge_ExpStp = 5;

namespace Robot {
enum Reference_Axis {
  X_Positive,
  X_Negative,
  Y_Positive,
  Y_Negative,
  Z_Positive,
  Z_Negative
};

enum class ShapeType{
    STP_Shape,
    IGS_Shape,
    MSH_Shape
};

enum class TargetSurface { Face_XY, Face_XZ, Face_YZ };

using X_Normal = std::pair<Base::Vector3d, Base::Vector3d>;
class CAD_Utility {
public:
  CAD_Utility();
  static bool isShapeValid(const TopoDS_Shape &t_shape);
  // Data extraction
  static std::vector<TopoDS_Face>
  getFacesFromDataSource(const App::PropertyLinkSub &t_selection);
  static std::vector<std::string>
  getFaceNameFromDataSource(const App::PropertyLinkSub &t_selection);
  static std::vector<std::string>
  getEdgeNameFromDataSource(const App::PropertyLinkSub &t_selection);

  static TopoDS_Face getTargetFaceObj(const App::PropertyLinkSub &t_selection,
                                      const std::string faceName);

  static std::vector<TopoDS_Edge>
  getEdgesFromDataSource(const App::PropertyLinkSub &t_selection);
  // Calculation
  static TopoDS_Edge getFaceIntersectionResult(const TopoDS_Face &f1,
                                               const TopoDS_Face &f2);
  static const Base::Placement calculateLinkedFaceCenter(const App::PropertyLinkSub &t_Face);
  static const Base::Placement calculateLinkefCurvCenter(const App::PropertyLinkSub &t_Edge);
  static const Base::Placement calculateSeletedFaceCenter(const TopoDS_Face &t_face,
                                                          Reference_Axis ref_Axis = Reference_Axis::Z_Positive);
  static Base::Placement getKeyPntOnEdge();
  static std::vector<Base::Placement> generateEdgeKeyPnts(const TopoDS_Edge &t_edge,
                                                          const double exp_stp,
                                                          const TopoDS_Face &f_1,
                                                          const TopoDS_Face &f_2,
                                                          bool face1_Reversed = false,
                                                          bool face2_Reversed = false,
                                                          bool direction_Reversed = false,
                                                          Reference_Axis ref_axis = Reference_Axis::Z_Positive);

  static std::vector<Base::Placement> removeColinearPoints(const std::vector<Base::Placement>& t_Pnts);

  static X_Normal calculateNormal2Face_fromEdge(const TopoDS_Edge &t_edge,
                                                 const Standard_Real &t_pos,
                                                 const TopoDS_Face &t_Face,
                                                 const double face_expStp);
  static gp_Vec getFaceAverageNormal(const TopoDS_Face &face);
  static X_Normal getAverageNorm(const X_Normal &norm_1,
                                  const X_Normal &norm_2);
  static Base::Placement getCurveCenterPnt(const TopoDS_Edge &t_edge);

  // For testing
  static std::vector<TopoDS_Edge> generate_TestEdges_onXY();
  static std::vector<TopoDS_Edge> generate_TestEdges_onXZ();
  static std::vector<TopoDS_Edge> generate_TestEdges_onYZ();
  static TopoDS_Edge generate_TestCurveOnXY();
  static TopoDS_Face generate_A_Face(TargetSurface t_surf);
  static TopoDS_Wire generate_A_Wire(TargetSurface t_surf);

private:
  static bool connectEdges(const std::list<TopoDS_Edge> &edges, TopoDS_Wire &wire);
};
}

#endif // CAD_UTILITY_H
