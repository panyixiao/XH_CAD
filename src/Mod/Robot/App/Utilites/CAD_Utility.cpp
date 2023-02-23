// Created by Yixiao 2017/05/08
#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <Adaptor3d_HSurface.hxx>
#include <Adaptor3d_IsoCurve.hxx>
#include <BRepAdaptor_HSurface.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp_Face.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAbs_IsoType.hxx>
#include <Geom_Surface.hxx>
//#include <OpenGl_tgl_funcs.hxx>
#include <Precision.hxx>
#include <TopAbs_State.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <GeomLProp_SLProps.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "CAD_Utility.h"
#include <Base/Console.h>
#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/App/TopoShape.h>

#include "Mod/Robot/App/PlanningObj/PlanningObject.h"
//#include <RD_DataStructure/RD_STL/RD_Vector.h>

using namespace Robot;
const double collisionFree_dis = 1;

// Assuming default direction is point to Z-positive
const Base::Vector3d reference_Axis_Z_Positive = Base::Vector3d(0, 0, 1);
const Base::Vector3d reference_Axis_Z_Negative = Base::Vector3d(0, 0, -1);
const Base::Vector3d reference_Axis_Y_Positive = Base::Vector3d(0, 1, 0);
const Base::Vector3d reference_Axis_Y_Negative = Base::Vector3d(0, -1, 0);
const Base::Vector3d reference_Axis_X_Positive = Base::Vector3d(1, 0, 0);
const Base::Vector3d reference_Axis_X_Negative = Base::Vector3d(-1, 0, 0);

CAD_Utility::CAD_Utility() {}


const Base::Placement CAD_Utility::calculateLinkedFaceCenter(const App::PropertyLinkSub &t_Face) {
  Base::Placement t_center = Base::Placement();
  if (t_Face.getValue() != nullptr) {
    auto selected_face = getFacesFromDataSource(t_Face);
    if (selected_face.empty() || selected_face.size() != 1) {
      Base::Console().Error("Warning(CAD_Utility): Empty Linked Face!\n");
      return t_center;
    }
    t_center = calculateSeletedFaceCenter(selected_face[0]);
  }
  return t_center;
}


std::vector<TopoDS_Face>
CAD_Utility::getFacesFromDataSource(const App::PropertyLinkSub &t_selection) {
  std::vector<TopoDS_Face> result;
  auto feature = t_selection.getValue();
  if (feature == nullptr) {
    Base::Console().Error("CAD_Utility: feature pointer is empty!\n");
    return result;
  }
  if (!feature->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId())) {
    Base::Console().Error(
        "CAD_Utility: Selected feature is not a Part Feature!\n");
    return result;
  }
  const Part::TopoShape &TopShape =
      static_cast<Part::Feature *>(feature)->Shape.getShape();
  if (TopShape.isNull()) {
    Base::Console().Error("CAD_Utility: Failed to get Shape from feature\n");
    return result;
  }
  auto faceNames = t_selection.getSubValuesStartsWith("Face");
  for (auto faceName : faceNames) {
    // Extract geometry from Reference Faces
    auto face_obj = TopoDS::Face(TopShape.getSubShape(faceName.c_str()));
    result.push_back(face_obj);
  }
  return result;
}

std::vector<TopoDS_Edge>
CAD_Utility::getEdgesFromDataSource(const App::PropertyLinkSub &t_selection) {
  std::vector<TopoDS_Edge> result;
  auto feature = t_selection.getValue();
  if (feature == nullptr) {
    Base::Console().Error("CAD_Utility: Feature pointer is nullptr!\n");
    return result;
  }
  if (!feature->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId())) {
    Base::Console().Error(
        "CAD_Utility: Selected feature is not a Part Feature!\n");
    return result;
  }
  const Part::TopoShape &TopShape = static_cast<Part::Feature *>(feature)->Shape.getShape();
  if (TopShape.isNull()) {
    Base::Console().Error("CAD_Utility: Failed to get Shape from feature!\n");
    return result;
  }
  auto edgeNames = t_selection.getSubValuesStartsWith("Edge");
  if (edgeNames.empty()) {
    return result;
  }
  for (auto edgeName : edgeNames) {
    result.push_back(TopoDS::Edge(TopShape.getSubShape(edgeName.c_str())));
  }
  return result;
}

std::vector<std::string> CAD_Utility::getFaceNameFromDataSource(
    const App::PropertyLinkSub &t_selection) {
  auto feature = t_selection.getValue();
  if (feature == nullptr ||
      !feature->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId())) {
    Base::Console().Message("Selected feature is empty!\n");
    return std::vector<std::string>();
  }
  return t_selection.getSubValuesStartsWith("Face");
}

std::vector<std::string> CAD_Utility::getEdgeNameFromDataSource(
    const App::PropertyLinkSub &t_selection) {
  auto feature = t_selection.getValue();
  if (feature == nullptr ||
      !feature->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId())) {
    Base::Console().Message("Selected feature is empty!\n");
    return std::vector<std::string>();
  }
  return t_selection.getSubValuesStartsWith("Edge");
}

TopoDS_Face
CAD_Utility::getTargetFaceObj(const App::PropertyLinkSub &t_selection,
                              const std::string faceName) {
  auto feature = t_selection.getValue();
  if (feature == nullptr ||
      !feature->getTypeId().isDerivedFrom(Part::Feature::getClassTypeId())) {
    Base::Console().Message("Selected feature is empty!\n");
    return TopoDS_Face();
  }
  const Part::TopoShape &TopShape =
      static_cast<Part::Feature *>(feature)->Shape.getShape();
  if (TopShape.isNull()) {
    Base::Console().Message("Failed to get Shape from feature\n");
    return TopoDS_Face();
  }
  auto faceNames = t_selection.getSubValuesStartsWith("Face");
  for (auto name : faceNames) {
    if (faceName == name)
      return TopoDS::Face(TopShape.getSubShape(faceName.c_str()));
  }
  return TopoDS_Face();
}

#include <BOPTools_AlgoTools3D.hxx>
#include <Standard_Real.hxx>
X_Normal CAD_Utility::calculateNormal2Face_fromEdge(const TopoDS_Edge &t_edge,
                                                    const Standard_Real &t_pos,
                                                    const TopoDS_Face &t_Face,
                                                    const double face_expStp) {
  X_Normal normal_ret;
  gp_Pnt aP;
  gp_Dir aD;

  /// Computes normal to the face <aF> for the 3D-point that belongs to the edge <aE> at parameter <aT>.
  /// Output:
  /// aPx - the 3D-point where the normal computed
  /// aD - the normal;
  /// Warning:
  /// The normal is computed not exactly in the point on the edge, but in point that is near to the edge towards to the face material (so, we'll have approx. normal);
  /// The point is computed using PointNearEdge function with the shifting value <aDt2D> from the edge;
  /// No checks on this value will be done.

  BOPTools_AlgoTools3D::GetApproxNormalToFaceOnEdge(t_edge, t_Face, t_pos, aP,
                                                    aD, face_expStp);
  normal_ret.first = Base::Vector3d(aP.X(), aP.Y(), aP.Z());
  normal_ret.second = Base::Vector3d(aD.X(), aD.Y(), aD.Z());

  gp_Pnt2d r_pnt2d;
  gp_Pnt r_pnt;
  BOPTools_AlgoTools3D::PointNearEdge(t_edge, t_Face, t_pos,
                                      BOPTools_AlgoTools3D::MinStepIn2d(),
                                      r_pnt2d, r_pnt);
  gp_Pnt a_rP;
  gp_Vec aD1U, aD1V;
  const Handle(Geom_Surface) &aS = BRep_Tool::Surface(t_Face);
  aS->D1(r_pnt2d.X(), r_pnt2d.Y(), a_rP, aD1U, aD1V);
  gp_Dir aDD1U(aD1U);
  gp_Dir aDD1V(aD1V);
  auto aDNS = aDD1U ^ aDD1V;
  if (t_Face.Orientation() == TopAbs_REVERSED) {
    aDNS.Reverse();
  }

  return normal_ret;
}

bool CAD_Utility::isShapeValid(const TopoDS_Shape &t_shape) {
  return !t_shape.IsNull();
}

const Base::Placement CAD_Utility::calculateLinkefCurvCenter(const App::PropertyLinkSub &t_Edge)
{
    auto curv_Vec = getEdgesFromDataSource(t_Edge);
    if(curv_Vec.empty()){
        Base::Console().Warning("Warning(CAD_Utility): No Edge Found From selected Feature!");
        return Base::Placement();
    }
}


const Base::Placement CAD_Utility::calculateSeletedFaceCenter(const TopoDS_Face &t_face,
                                                              Reference_Axis ref_Axis) {
  GProp_GProps massProps;
  BRepGProp::SurfaceProperties(t_face, massProps);
  auto cPnt_Face = massProps.CentreOfMass();
  auto cVec_Face = getFaceAverageNormal(t_face);
  Base::Placement face_Center;
  double f_x, f_y, f_z;
  f_x = cPnt_Face.X() + cVec_Face.X() * collisionFree_dis;
  f_y = cPnt_Face.Y() + cVec_Face.Y() * collisionFree_dis;
  f_z = cPnt_Face.Z() + cVec_Face.Z() * collisionFree_dis;
  face_Center.setPosition(Base::Vector3d(f_x, f_y, f_z));
  auto face_dir = Base::Vector3d(cVec_Face.X(), cVec_Face.Y(), cVec_Face.Z());
  switch (ref_Axis) {
  case Reference_Axis::X_Negative:
    face_Center.setRotation(
        Base::Rotation(reference_Axis_X_Negative, face_dir));
    break;
  case Reference_Axis::X_Positive:
    face_Center.setRotation(
        Base::Rotation(reference_Axis_X_Positive, face_dir));
    break;
  case Reference_Axis::Y_Negative:
    face_Center.setRotation(
        Base::Rotation(reference_Axis_Z_Negative, face_dir));
    break;
  case Reference_Axis::Y_Positive:
    face_Center.setRotation(
        Base::Rotation(reference_Axis_Y_Positive, face_dir));
    break;
  case Reference_Axis::Z_Negative:
    face_Center.setRotation(
        Base::Rotation(reference_Axis_Z_Negative, face_dir));
    break;
  case Reference_Axis::Z_Positive:
    face_Center.setRotation(
        Base::Rotation(reference_Axis_Z_Positive, face_dir));
    break;
  }
  return face_Center;
}

#include <Base/Sequencer.h>
#include <BRepAdaptor_Curve.hxx>
#include <CPnts_AbscissaPoint.hxx>
#include <GeomAbs_CurveType.hxx>
std::vector<Base::Placement>
CAD_Utility::generateEdgeKeyPnts(const TopoDS_Edge &t_edge,
                                 const double exp_stp,
                                 const TopoDS_Face &f_1,
                                 const TopoDS_Face &f_2,
                                 bool face1_Reversed,
                                 bool face2_Reversed, bool direction_Reversed,
                                 Reference_Axis ref_axis) {
  std::vector<Base::Placement> keyPnts;
  if (t_edge.IsNull() || f_1.IsNull() || f_2.IsNull())
    return keyPnts;

  BRepAdaptor_Curve adapter(t_edge);
  auto keyPnt_Generator = [adapter, t_edge, f_1, f_2, exp_stp, face1_Reversed, face2_Reversed, ref_axis]
                          (Standard_Real t_Edge_pos, bool orientation_Pnt = false) -> Base::Placement
  {
    gp_Pnt Pnt = adapter.Value(t_Edge_pos);
    auto face_expStp = exp_stp > 3 ? exp_stp / 3 : exp_stp;
    auto norm_Pnt2f1 = calculateNormal2Face_fromEdge(t_edge, t_Edge_pos, f_1, face_expStp);
    if (face1_Reversed)
      norm_Pnt2f1.second *= (-1);
    auto norm_Pnt2f2 = calculateNormal2Face_fromEdge(t_edge, t_Edge_pos, f_2, face_expStp);
    if (face2_Reversed)
      norm_Pnt2f2.second *= (-1);
    auto norm_Pnt = getAverageNorm(norm_Pnt2f1, norm_Pnt2f2);
    double kp_x, kp_y, kp_z;
    double shift_distance = 0.0;
    if (orientation_Pnt)
        shift_distance = 30;
    else
        shift_distance = collisionFree_dis;

    kp_x = Pnt.X() - norm_Pnt.second.x * shift_distance;
    kp_y = Pnt.Y() - norm_Pnt.second.y * shift_distance;
    kp_z = Pnt.Z() - norm_Pnt.second.z * shift_distance;
    Base::Placement keyPnt;
    keyPnt.setPosition(Base::Vector3d(kp_x, kp_y, kp_z));

    Base::Vector3d reference_axis;
    switch (ref_axis) {
    case Reference_Axis::X_Negative:
      reference_axis = reference_Axis_X_Negative;
      break;
    case Reference_Axis::X_Positive:
      reference_axis = reference_Axis_X_Positive;
      break;
    case Reference_Axis::Y_Negative:
      reference_axis = reference_Axis_Y_Negative;
      break;
    case Reference_Axis::Y_Positive:
      reference_axis = reference_Axis_Y_Positive;
      break;
    case Reference_Axis::Z_Negative:
      reference_axis = reference_Axis_Z_Negative;
      break;
    case Reference_Axis::Z_Positive:
      reference_axis = reference_Axis_Z_Positive;
      break;
    default:
      reference_axis = reference_Axis_Z_Positive;
      break;
    }
    keyPnt.setRotation(Base::Rotation(reference_axis, norm_Pnt.second));
    return keyPnt;
  };

  auto edgeType = adapter.GetType();
  switch (edgeType) {
  case GeomAbs_Line:
  case GeomAbs_Ellipse:
  case GeomAbs_Hyperbola:
  case GeomAbs_Parabola:
  case GeomAbs_BezierCurve:
  case GeomAbs_OtherCurve:
  case GeomAbs_BSplineCurve: {
    Standard_Real Length = CPnts_AbscissaPoint::Length(adapter);
    Standard_Real ParLength = adapter.LastParameter() - adapter.FirstParameter();
    Standard_Real NbrSegments = Round(Length / exp_stp);
    Standard_Real beg = adapter.FirstParameter();
    Standard_Real end = adapter.LastParameter();
    Standard_Real stp = ParLength / NbrSegments;

    if (direction_Reversed) {
      std::swap(beg, end);
      stp = -stp;
    }

    Base::SequencerLauncher seq("Create way points",
                                static_cast<size_t>((end - beg) / stp));
    auto pos = beg;
    auto pnt_s = keyPnt_Generator(beg, true);
    keyPnts.push_back(pnt_s);
    pos+=stp;
    if (direction_Reversed) {
      for (; pos > end; pos += stp) {
        auto pnt = keyPnt_Generator(pos);
        keyPnts.push_back(pnt);
        seq.next();
      }
    } else {
      for (; pos < end; pos += stp) {
        auto pnt = keyPnt_Generator(pos);
        keyPnts.push_back(pnt);
        seq.next();
      }
    }
    auto pnt_f = keyPnt_Generator(end, true);
    keyPnts.push_back(pnt_f);
  } break;
  case GeomAbs_Circle: {
    Standard_Real Length = CPnts_AbscissaPoint::Length(adapter);
    Standard_Real ParLength =
        adapter.LastParameter() - adapter.FirstParameter();
    Standard_Real NbrSegments = Round(Length / exp_stp);
    Standard_Real SegLength = ParLength / NbrSegments;
    auto beg_pos = adapter.FirstParameter();
    auto end_pos = adapter.LastParameter();
    auto trav_pos = beg_pos;

    if (t_edge.Orientation() == TopAbs_REVERSED) {
      trav_pos = end_pos;
      SegLength = -SegLength;
      // Insert Orientation Adjustment Pnt in Beginning
      auto pnt_s = keyPnt_Generator(trav_pos, true);
      keyPnts.push_back(pnt_s);
      for (; trav_pos > beg_pos; trav_pos += SegLength) {
        auto pnt = keyPnt_Generator(trav_pos);
        keyPnts.push_back(pnt);
      }
      // Insert Orientation Adjustment Pnt in the End
      auto pnt_f = keyPnt_Generator(trav_pos - SegLength, true);
      keyPnts.push_back(pnt_f);
    } else {
      // Insert Orientation Adjustment Pnt in Beginning
      auto pnt_s = keyPnt_Generator(trav_pos, true);
      keyPnts.push_back(pnt_s);
      for (; trav_pos < end_pos; trav_pos += SegLength) {
        auto pnt = keyPnt_Generator(trav_pos);
        keyPnts.push_back(pnt);
      }
      // Insert Orientation Adjustment Pnt in the End
      auto pnt_f = keyPnt_Generator(trav_pos - SegLength, true);
      keyPnts.push_back(pnt_f);
    }

  } break;
  default:
    break;
  }
  return keyPnts;
}

std::vector<Base::Placement> CAD_Utility::removeColinearPoints(const std::vector<Base::Placement> &t_Pnts)
{
    std::vector<Base::Placement> result;
    if(t_Pnts.empty())
        return result;
    Base::Placement _refPoint, _2ndPoint, _3rdPoint;
    size_t ref_ID = 0, cmp_ID = 0;
    result.push_back(t_Pnts.front());
    for(cmp_ID = 1; cmp_ID+1< t_Pnts.size(); cmp_ID++){
        _2ndPoint = t_Pnts[cmp_ID];
        _3rdPoint = t_Pnts[cmp_ID+1];
        auto refVec = _2ndPoint.getPosition() - _refPoint.getPosition();
        auto cmpVec = _3rdPoint.getPosition() - _2ndPoint.getPosition();
        auto angle = refVec.GetAngle(cmpVec)*180.0/M_PI;
        if(angle > 0.5){
            // No-Colinear
            ref_ID = cmp_ID;
            _refPoint = t_Pnts[ref_ID];
            result.push_back(_refPoint);
        }
    }
    result.push_back(t_Pnts[t_Pnts.size()-1]);
    return result;
}

gp_Vec CAD_Utility::getFaceAverageNormal(const TopoDS_Face &t_face) {
  Handle(Geom_Surface) Surf = BRep_Tool::Surface(t_face);
  TopExp_Explorer edge_explorer;
  edge_explorer.Init(t_face, TopAbs_EDGE);
  TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
  double first, last;
  Handle(Geom2d_Curve) Curve =
      BRep_Tool::CurveOnSurface(edge, t_face, first, last);
  gp_Pnt2d P2d;
  Curve->D0(first, P2d);
  GeomLProp_SLProps SLProps(Surf, 1, 0.1);
  SLProps.SetParameters(P2d.X(), P2d.Y());
  gp_Dir direc = SLProps.Normal();
  gp_Vec normal(direc.X(), direc.Y(), direc.Z());
  TopAbs_Orientation orient = t_face.Orientation();
  // normal is oriented inside the shape ->reoriente it!
  if (orient == TopAbs_REVERSED) {
    return normal.Reversed().Normalized();
  }
  return normal.Normalized();

  //    BRepAdaptor_Surface aSurface(aFace);
  //    Standard_Real u1,u2,v1,v2;
  //    u1 = aSurface.FirstUParameter();
  //    u2 = aSurface.LastUParameter();
  //    v1 = aSurface.FirstVParameter();
  //    v2 = aSurface.LastVParameter();
  //    gp_Pnt aCenterOfFace;
  //    gp_Vec aVec1,aVec2,norm_atPnt;
  //    aSurface.D1((u1+u2)/2,(v1+v2)/2,aCenterOfFace,aVec1,aVec2);
  //    norm_atPnt = aVec1^aVec2;
}

#include <Approx_Curve3d.hxx>
#include <BRepAdaptor_HCompCurve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GeomAbs_Shape.hxx>
//#include <Handle_BRepAdaptor_HCompCurve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
TopoDS_Edge CAD_Utility::getFaceIntersectionResult(const TopoDS_Face &f1,
                                                   const TopoDS_Face &f2) {
  BRepAlgoAPI_Section face_intesec(f1, f2);
  if (face_intesec.IsDone()) {
    std::list<TopoDS_Edge> edges;
    TopExp_Explorer xp;
    for (xp.Init(face_intesec.Shape(), TopAbs_EDGE); xp.More(); xp.Next()) {
      edges.push_back(TopoDS::Edge(xp.Current()));
    }
    if (edges.empty()) {
      Base::Console().Error(
          "Face Intersection Complete, No valid Edge was found!\n");
      return TopoDS_Edge();
    } else if (edges.size() == 1) {
      return TopoDS_Edge(edges.front());
    } else {
      TopoDS_Wire result;
      Base::Console().Message("Connecting %d edges... \n", edges.size());
      connectEdges(edges, result);
      BRepAdaptor_CompCurve wireAdaptor(result);
      Handle_BRepAdaptor_HCompCurve curve =
          new BRepAdaptor_HCompCurve(wireAdaptor);
      // approximate the curve using a tolerance
      Approx_Curve3d approx(curve, 0.001, GeomAbs_C2, 200, 12);
      if (approx.IsDone() && approx.HasResult()) {
        // have the result
        Handle_Geom_Curve anApproximatedCurve = approx.Curve();
        return TopoDS_Edge(BRepBuilderAPI_MakeEdge(anApproximatedCurve));
      }
    }
  }
  Base::Console().Error("Face Intersection Calculation Error, Edge undefined!");
  return TopoDS_Edge();
}

//#include  < GeomAPI_IntSS.hxx>
// TopoDS_Shape AddNewIntersectSrf(const TopoDS_Face& srf1,const TopoDS_Face&
// srf2)
//{
// Handle(Geom_Surface) S1 = BRep_Tool::Surface(srf1);
// Handle(Geom_Surface) S2 = BRep_Tool::Surface(srf2);
////This class is instantiated as follows:
// GeomAPI_IntSS Intersector(S1, S2, Precision::Intersection());
////Once the GeomAPI_IntSS object has been created, it can be interpreted.
////Calling the number of intersection curves
// Standard_Integer nb = Intersector.NbLines();
////Calling the intersection curves
// if (nb > 0) {
// Handle(Geom_Curve) C = Intersector.Line(1);
////where Index is an integer between 1 and Nb.
// BRepBuilderAPI_MakeEdge
// edgebuilder(C,C->FirstParameter(),C->LastParameter());
// return edgebuilder.Shape();
//}
//}

X_Normal CAD_Utility::getAverageNorm(const X_Normal &norm_1, const X_Normal &norm_2) {
  X_Normal norm_ret;
  norm_ret.first = Base::Vector3d((norm_1.first + norm_2.first) / 2);
//  norm_ret.second = Base::Vector3d((norm_1.second + norm_2.second).Normalize() * (-1));
  norm_ret.second = Base::Vector3d((norm_1.second + norm_2.second)/2) * (-1);
  return norm_ret;
}

//#include <Geom_Circle.hxx>
//#include <Handle_Geom_Circle.hxx>
//#include <Standard_Type.hxx>
//#include <gp_Ax1.hxx>
//#include <gp_Circ.hxx>
// Base::Placement CAD_Utility::getCurveCenterPnt(const TopoDS_Edge &t_edge) {
//  Base::Placement result;
//  double first, last;
//  Handle(Geom_Curve) r_Curve = BRep_Tool::Curve(t_edge, first, last);
//  Handle(Standard_Type) type = r_Curve->DynamicType();
//  if (type == STANDARD_TYPE(Geom_Circle)) {
//    Handle(Geom_Circle) r_Circle = Handle_Geom_Circle::DownCast(r_Curve);
//    gp_Circ Circ = r_Circle->Circ();
//    gp_Pnt centre = Circ.Location();
//    result.setPosition(Base::Vector3d(centre.X(), centre.Y(), centre.Z()));
//    gp_Ax1 axe = Circ.Axis();
//    gp_Dir dir = axe.Direction();
//    //
//    result.setRotation(Base::Rotation(reference_Vec,Base::Vector3d(dir.X(),dir.Y(),dir.Z())));
//    double rayon;
//    rayon = Circ.Radius();
//  }
//  return result;
//}

#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_WireError.hxx>
bool CAD_Utility::connectEdges(const std::list<TopoDS_Edge> &edges,
                               TopoDS_Wire &wire) {
  std::list<TopoDS_Edge> edge_list = edges;
  bool found = false;
  while (edge_list.size() > 0) {
    BRepBuilderAPI_MakeWire mkWire;
    // add and erase first edge
    mkWire.Add(edge_list.front());
    edge_list.erase(edge_list.begin());
    wire = mkWire.Wire();
    // current new wire
    // try to connect each edge to the wire, the wire is complete if no more
    // egdes are connectible
    int connectedEdge = 0;
    do {
      found = false;
      for (std::list<TopoDS_Edge>::iterator pE = edge_list.begin();
           pE != edge_list.end(); ++pE) {
        mkWire.Add(*pE);
        connectedEdge++;
        if (mkWire.Error() != BRepBuilderAPI_DisconnectedWire) {
          // edge added ==> remove it from list
          found = true;
          edge_list.erase(pE);
          wire = mkWire.Wire();
          break;
        } else {
          Base::Console().Message(
              "Wire has disconnected edges! Only %d edges were connected\n",
              connectedEdge);
          return false;
        }
      }
    } while (found);
  }
  return found;
}

#include <Geom_Circle.hxx>
//#include <Handle_Geom_Circle.hxx>
#include <Standard_Type.hxx>
#include <gp_Ax1.hxx>
#include <gp_Circ.hxx>
Base::Placement CAD_Utility::getCurveCenterPnt(const TopoDS_Edge &t_curve) {
  Base::Placement result;
  double first, last;
  Handle(Geom_Curve) r_Curve = BRep_Tool::Curve(t_curve, first, last);
  Handle(Standard_Type) type = r_Curve->DynamicType();
  if (type == STANDARD_TYPE(Geom_Circle)) {
    Handle(Geom_Circle) r_Circle = Handle_Geom_Circle::DownCast(r_Curve);
    gp_Circ Circ = r_Circle->Circ();
    gp_Pnt centre = Circ.Location();
    result.setPosition(Base::Vector3d(centre.X(), centre.Y(), centre.Z()));
    gp_Ax1 axe = Circ.Axis();
    gp_Dir dir = axe.Direction();
    //        result.setRotation(Base::Rotation(reference_Vec,Base::Vector3d(dir.X(),dir.Y(),dir.Z())));
    double rayon;
    rayon = Circ.Radius();
  }
  return result;
}

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BoundedCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
// void CAD_Utility::generateShapeTest() {
//  TColgp_Array1OfPnt CurvePoles(1, 6);
//  gp_Pnt pt = gp_Pnt(0., 0., 0.);
//  CurvePoles(1) = pt;
//  pt = gp_Pnt(20., 50., 0.);
//  CurvePoles(2) = pt;
//  pt = gp_Pnt(60., 100., 0.);
//  CurvePoles(3) = pt;
//  pt = gp_Pnt(150., 0., 0.);
//  CurvePoles(4) = pt;
//  Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(CurvePoles);
//  TopoDS_Edge E = BRepBuilderAPI_MakeEdge(curve);
//  TopoDS_Wire W = BRepBuilderAPI_MakeWire(E);
//  gp_Circ c = gp_Circ(gp_Ax2(gp_Pnt(0., 0., 0.), gp_Dir(0., 1., 0.)), 10.);
//  TopoDS_Edge Ec = BRepBuilderAPI_MakeEdge(c);
//  TopoDS_Wire Wc = BRepBuilderAPI_MakeWire(Ec);
//  TopoDS_Face F = BRepBuilderAPI_MakeFace(gp_Pln(gp::ZOX()), Wc);
//  TopoDS_Shape S = BRepOffsetAPI_MakePipe(W, F);
//}

std::vector<TopoDS_Edge> CAD_Utility::generate_TestEdges_onXY() {
  std::vector<TopoDS_Edge> edges;
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 0., 0.), gp_Pnt(50., 0., 0)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(50., 0., 0.), gp_Pnt(50., 50., 0)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(50., 50., 0.), gp_Pnt(0., 50., 0)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 50., 0.), gp_Pnt(0., 0., 0)));
  return edges;
}

std::vector<TopoDS_Edge> CAD_Utility::generate_TestEdges_onXZ() {
  std::vector<TopoDS_Edge> edges;
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 0., 0.), gp_Pnt(50., 0., 0)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(50., 0., 0.), gp_Pnt(50., 0., 50)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(50., 0., 50.), gp_Pnt(0., 0., 50)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 0., 50.), gp_Pnt(0., 0., 0)));
  return edges;
}

std::vector<TopoDS_Edge> CAD_Utility::generate_TestEdges_onYZ() {
  std::vector<TopoDS_Edge> edges;
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 0., 0.), gp_Pnt(0., 50., 0)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 50., 0.), gp_Pnt(0., 50., 50)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 50., 50.), gp_Pnt(0., 0., 50)));
  edges.push_back(
      BRepBuilderAPI_MakeEdge(gp_Pnt(0., 0., 50.), gp_Pnt(0., 0., 0)));
  return edges;
}

TopoDS_Edge CAD_Utility::generate_TestCurveOnXY() {
  return BRepBuilderAPI_MakeEdge(gp_Circ(
      gp_Ax2(gp_Pnt(25., 0., 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 25.0));
}

TopoDS_Wire CAD_Utility::generate_A_Wire(TargetSurface t_surf) {
  BRepBuilderAPI_MakeWire result_wire;
  switch (t_surf) {
  case TargetSurface::Face_XY:
    for (auto edge : generate_TestEdges_onXY()) {
      result_wire.Add(edge);
    }
    break;
  case TargetSurface::Face_XZ:
    for (auto edge : generate_TestEdges_onXZ()) {
      result_wire.Add(edge);
    }
    break;
  case TargetSurface::Face_YZ:
    for (auto edge : generate_TestEdges_onYZ()) {
      result_wire.Add(edge);
    }
    break;
  default:
    break;
  }
  return result_wire.Wire();
}

TopoDS_Face CAD_Utility::generate_A_Face(TargetSurface t_surf) {
  TopoDS_Face result_Face;
  switch (t_surf) {
  case TargetSurface::Face_XY:
    result_Face =
        BRepBuilderAPI_MakeFace(gp_Pln(gp::XOY()), generate_A_Wire(t_surf));
    break;
  case TargetSurface::Face_XZ:
    result_Face =
        BRepBuilderAPI_MakeFace(gp_Pln(gp::ZOX()), generate_A_Wire(t_surf));
    break;
  case TargetSurface::Face_YZ:
    result_Face =
        BRepBuilderAPI_MakeFace(gp_Pln(gp::YOZ()), generate_A_Wire(t_surf));
    break;
  default:
    break;
  }
  return result_Face;
}
