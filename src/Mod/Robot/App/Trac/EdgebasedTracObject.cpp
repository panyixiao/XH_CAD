/***************************************************************************
 *   Copyright (c) 2010 Jürgen Riegel (juergen.riegel@web.de)              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "EdgebasedTracObject.h"
//#include <App/DocumentObjectPy.h>
//#include <Base/Placement.h>
#include <Base/Sequencer.h>
#include <Base/Console.h>
#include <Mod/Part/App/edgecluster.h>
#include <Mod/Part/App/PartFeature.h>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <CPnts_AbscissaPoint.hxx>
#include <TopExp.hxx>

#include "RobotWaypoint.h"
#include "RobotTracObject.h"
#include "Mod/Robot/App/Utilites/CAD_Utility.h"

using namespace Robot;
using namespace App;

PROPERTY_SOURCE(Robot::EdgebasedTracObject, Robot::RobotTracObject)


EdgebasedTracObject::EdgebasedTracObject()
{

    ADD_PROPERTY_TYPE(FaceSource,      (0)  , "DataSource",Prop_None,"Face References to calculate Trac");
    ADD_PROPERTY_TYPE(EdgeSource,      (0)  , "DataSource",Prop_None,"Edge References to calculate Trac");
    ADD_PROPERTY_TYPE(UseRotation, (0)  , "Property",Prop_None,"use orientation of the edge");
    ADD_PROPERTY_TYPE(InterpoDist, (5.0), "Property", Prop_None, "Distance value for interpolation");
//    ADD_PROPERTY_TYPE( PoseAdjust, (Base::Placement()), "Property", Prop_None, "Pose adjustment for edge");
    ADD_PROPERTY_TYPE(Accelerate,(0.0), "Trac Parameters", Prop_None, "Accelerate setting for Trac");
    ADD_PROPERTY_TYPE(Angle_Roll, (0.0), "Trac Parameters", Prop_None, "Edge Trac Rolling Angle");
    ADD_PROPERTY_TYPE(Angle_Pitch, (0.0), "Trac Parameters", Prop_None, "Edge Trac Pitching Angle");
    ADD_PROPERTY_TYPE(Angle_Yaw, (0.0), "Trac Parameters", Prop_None, "Edge Trac Yawing Angle");
    ADD_PROPERTY_TYPE(Offset_Z, (0.0), "Trac Parameters", Prop_None, "Edge Trac Offset along Edge Coord Z axis");
    ADD_PROPERTY_TYPE(Offset_W, (0.0), "Trac Parameters", Prop_None, "Edge Trac Offset along Edge Reference Wall");
    ADD_PROPERTY_TYPE(Offset_B, (0.0), "Trac Parameters", Prop_None, "Edge Trac Offset along Edge Reference Base");

    // Weld Param
    ADD_PROPERTY_TYPE(TorchToolID,(1),"Weld Parameters", Prop_None, "Torch Tool ID");
    ADD_PROPERTY_TYPE(WeldSpeed, (0.0), "Weld Parameters", Prop_None, "Welding Speed setting for Trac");
    ADD_PROPERTY_TYPE(ApproachSpeed, (0.0), "Weld Parameters", Prop_None, "Lead-in/out Speed setting for Trac");
    // Scan Param
    ADD_PROPERTY_TYPE(ScanToolID,(2),"Scan Trac Parameters", Prop_None, "Scan Tool ID");
    ADD_PROPERTY_TYPE(ScanSpeed, (0.0), "Scan Trac Parameters", Prop_None, "Scaning Speed setting for Trac");

    // Scan Param

    ADD_PROPERTY(Reverse_EdgeDir, (false));
    Reverse_EdgeDir.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(Reverse_RefFace_1,(false));
    Reverse_RefFace_1.setStatus(App::Property::Status::Hidden, true);
    ADD_PROPERTY(Reverse_RefFace_2,(false));
    Reverse_RefFace_2.setStatus(App::Property::Status::Hidden, true);

    ScanSpeed.setValue(200.0);
    WeldSpeed.setValue(20.0);
    Accelerate.setValue(50.0);

    NbrOfEdges = 0;
    NbrOfCluster = 0;

    PropertyFloatConstraint::Constraints* c = new PropertyFloatConstraint::Constraints();
    c->LowerBound = -60.0;
    c->UpperBound = 60.0;
    c->StepSize = 0.5;
    Angle_Roll.setConstraints(c);
    Angle_Pitch.setConstraints(c);
}

EdgebasedTracObject::~EdgebasedTracObject()
{
}

bool EdgebasedTracObject::updateReferences()
{
    // Faces
    auto faceNames = CAD_Utility::getFaceNameFromDataSource(FaceSource);
    if(faceNames.empty()){
        Base::Console().Error("EdgeConstraint: Reference Face is empty!\n");
        return false;
    }
    else if(faceNames.size()!=2){
        Base::Console().Error("EdgeConstraint: More/Less than 2 faces as reference faces!\n");
        return false;
    }

    // Edges
    auto edgeName = CAD_Utility::getEdgeNameFromDataSource(EdgeSource);
    if(edgeName.empty() || edgeName.size()!=1){
        Base::Console().Message("EdgeConstraint: No Valid Edge was selected!\n");
    }
    return true;
}

void EdgebasedTracObject::setTracSafePoint(const GroupPose &t_Pose)
{
    m_TracSafePoint = t_Pose;
}

bool EdgebasedTracObject::generateConstraint(const uint t_ID, bool newRef)
{
    bool success;
    if(t_ID == 1){
        if(edge1_Poses.empty()||newRef){
            edge1_FaceRef = CAD_Utility::getFacesFromDataSource(FaceSource);
            edge1_EdgeRef = CAD_Utility::getEdgesFromDataSource(EdgeSource);
        }
        float interpoLen = InterpoDist.getValue();
//        if(!edge2_Poses.empty()&&!edge1_Poses.empty()){
//            auto len_1 = (float)edge1_Poses.size();
//            auto len_2 = (float)edge2_Poses.size();
//            float ratio = len_1>len_2?(len_2/len_1):(len_1/len_2);
//            interpoLen*=ratio;
//        }
        edge1_Poses = generateEdgePoses(edge1_FaceRef,edge1_EdgeRef,interpoLen);
        success = !edge1_Poses.empty();
    }
    else if(t_ID == 2){
        if(edge2_Poses.empty()||newRef){
            edge2_FaceRef = CAD_Utility::getFacesFromDataSource(FaceSource);
            edge2_EdgeRef = CAD_Utility::getEdgesFromDataSource(EdgeSource);
        }
        float interpoLen = InterpoDist.getValue();
//        if(!edge2_Poses.empty()&&!edge1_Poses.empty()){
//            auto len_1 = (float)edge1_Poses.size();
//            auto len_2 = (float)edge2_Poses.size();
//            float ratio = len_1>len_2?(len_2/len_1):(len_1/len_2);
//            interpoLen*=ratio;
//        }
        edge2_Poses = generateEdgePoses(edge2_FaceRef,edge2_EdgeRef,interpoLen);
        success = !edge2_Poses.empty();
    }
    return success;
}

bool EdgebasedTracObject::generateProgram(const uint t_ID)
{
    std::vector<Base::Placement> t_Poses;
    std::string executorName = ExecutorName.getStrValue();
    switch(t_ID){
    case 1:
        t_Poses = edge1_Poses;
        break;
    case 2:
        t_Poses = edge2_Poses;
        break;
    }
    if(t_Poses.empty())
        return false;
    resetData();
    ToolType t_ToolType;
    float trac_speed = 0.0;
    switch(m_TYPE){
    case TracType::SCANTRAC:
        t_ToolType = ToolType::_2DScanner;
        insertCMD_SwitchTool(executorName,t_ToolType);
        trac_speed = ScanSpeed.getValue();
        break;
    case TracType::SEAMTRAC:
        t_ToolType = ToolType::WeldTorch;
        insertCMD_SwitchTool(executorName,t_ToolType);
        trac_speed = WeldSpeed.getValue();
        break;
    }
    // To Safe Point
    RobotWaypoint safePoint(m_TracSafePoint);
    insertCMD_MOVE(ExecutorName.getStrValue(),
                   safePoint,
                   Robot::MoveType::MOVL,
                   Robot::MovePrec::FINE,
                   trac_speed);

    for(int i = 0; i<t_Poses.size(); i++){
        RobotWaypoint newWaypoint(t_Poses[i],
                                  CurrentExternalAxis.getValues(),
                                  t_ID);
        if(i == 0){
            // To first Point
            insertCMD_MOVE(ExecutorName.getStrValue(),
                           newWaypoint,
                           Robot::MoveType::MOVL,
                           Robot::MovePrec::FINE,
                           trac_speed);
            insertCMD_SetToolActivate(executorName,t_ToolType);

        }
        else if(i == t_Poses.size()-1){
            insertCMD_MOVE(ExecutorName.getStrValue(),
                           newWaypoint,
                           Robot::MoveType::MOVL,
                           Robot::MovePrec::FINE,
                           trac_speed);
            insertCMD_SetToolDeActivate(executorName,t_ToolType);
            // Go Back To Safe Point
            insertCMD_MOVE(ExecutorName.getStrValue(),
                           safePoint,
                           Robot::MoveType::MOVL,
                           Robot::MovePrec::FINE,
                           trac_speed);
        }
        else
            insertCMD_MOVE(executorName,
                           newWaypoint,
                           Robot::MoveType::MOVL,
                           Robot::MovePrec::CNT,
                           trac_speed,5.0);
    }
    return true;
}

bool EdgebasedTracObject::regenerateProgram()
{
    // Get Edges from linked Source
    auto selected_Faces = CAD_Utility::getFacesFromDataSource(FaceSource);
    auto selected_Edges = CAD_Utility::getEdgesFromDataSource(EdgeSource);
    // container for all the edges
    std::vector<TopoDS_Edge> edges;
    if(selected_Faces.size() == 2){
        auto edge = CAD_Utility::getFaceIntersectionResult(selected_Faces[0],
                                                           selected_Faces[1]);
        edges.push_back(edge);
    }
    else if(!selected_Edges.empty()){
        edges = selected_Edges;
    }

    // instanciate a edge cluster sorter and get the result
    Part::Edgecluster acluster(edges);
    Part::tEdgeClusterVector aclusteroutput = acluster.GetClusters();
    if(aclusteroutput.size() == 0){
        Base::Console().Warning("EdgebasedTracObject: Edgecluster Found No Edges specified\n");
        return false;
    }
    // Clear Commands & Movement
    resetData();


    // Change Tool
    ToolType t_Type;
    float trac_speed = 0.0;
    switch(m_TYPE){
    case TracType::SCANTRAC:
        t_Type = ToolType::_2DScanner;
        trac_speed = ScanSpeed.getValue();
        break;
    case TracType::SEAMTRAC:
        t_Type = ToolType::WeldTorch;
        trac_speed = WeldSpeed.getValue();
        break;
    default:
        break;
    }
    insertCMD_SwitchTool(ExecutorName.getStrValue(),t_Type);
    insertCMD_SetToolActivate(ExecutorName.getStrValue(),t_Type);

    // set the number of cluster and edges
    NbrOfCluster = aclusteroutput.size();
    NbrOfEdges = 0;
    for(std::vector<std::vector<TopoDS_Edge> >::const_iterator it=aclusteroutput.begin();it!=aclusteroutput.end();++it)
        NbrOfEdges += it->size();

    // cycle trough the cluster
    for(std::vector<std::vector<TopoDS_Edge> >::const_iterator it=aclusteroutput.begin();it!=aclusteroutput.end();++it)
    {
        // cycle through the edges of the cluster
        for(std::vector<TopoDS_Edge>::const_iterator it2=it->begin();it2!=it->end();++it2)
        {
           auto edgePntPoses = CAD_Utility::generateEdgeKeyPnts(*it2,
                                                                InterpoDist.getValue(),
                                                                selected_Faces[0],
                                                                selected_Faces[1],
                                                                Reverse_RefFace_1.getValue(),
                                                                Reverse_RefFace_2.getValue(),
                                                                Reverse_EdgeDir.getValue());
//           if(!edgePntPoses.empty()){
//               for(std::size_t i = 0; i<edgePntPoses.size(); i++){
//                   RobotWaypoint newWaypoint(edgePntPoses[i]);
//                   if(i == 0 || i == edgePntPoses.size()-1)
//                       insertCMD_MOVE(ExecutorName.getStrValue(),
//                                      newWaypoint,
//                                      Robot::MoveType::MOVL,
//                                      Robot::MovePrec::FINE,
//                                      trac_speed);
//                   else
//                       insertCMD_MOVE(ExecutorName.getStrValue(),
//                                      newWaypoint,
//                                      Robot::MoveType::MOVL,
//                                      Robot::MovePrec::CNT,
//                                      trac_speed,3.0);
//               }
//           }
        }
    }

    insertCMD_SetToolDeActivate(ExecutorName.getStrValue(),t_Type);
    return true;
}

void EdgebasedTracObject::setEdgeTracType(const TracType &t_Type)
{
    if(t_Type ==  TracType::SEAMTRAC){
        WeldSpeed.setStatus(App::Property::Status::Hidden, true);
        ScanSpeed.setStatus(App::Property::Status::Hidden, false);
    }else{
        ScanSpeed.setStatus(App::Property::Status::Hidden, true);
        WeldSpeed.setStatus(App::Property::Status::Hidden, false);
    }
    m_TYPE = t_Type;
}

void EdgebasedTracObject::cutEdgeAtPoint(const uint position, bool abandonRest)
{
    auto waypointVec = m_ProgramPtr->getWaypointData();
    if(position>waypointVec.size())
        return;
    if(abandonRest){
        for(int i = position; i<waypointVec.size(); i++){
            waypointVec.erase(waypointVec.begin()+i);
        }
    }
    else{
        for(int i = 0; i<position; i++){
            waypointVec.erase(waypointVec.begin()+i);
        }
    }
    m_ProgramPtr->replacePoseData(waypointVec);
}

App::DocumentObjectExecReturn *EdgebasedTracObject::execute(void)
{
    return App::DocumentObject::StdReturn;
}

void EdgebasedTracObject::onChanged(const Property* prop)
{
    if(prop == &InterpoDist){
        auto min_Radius = InterpoDist.getValue()-0.5;
        if(min_Radius<0)
            min_Radius = 1.0;
        m_ProgramPtr->setRoundupDist(min_Radius);
    }
    App::GeoFeature::onChanged(prop);
}

std::vector<Base::Placement> EdgebasedTracObject::generateEdgePoses(const std::vector<TopoDS_Face> faceRefs,
                                                                    const std::vector<TopoDS_Edge> edgeRefs,
                                                                    const float interpoLen)
{
    std::vector<Base::Placement> calculatedPoses;
    // Get Edges from linked Source
    auto selected_Faces = faceRefs;
    auto selected_Edges = edgeRefs;
    // container for all the edges
    std::vector<TopoDS_Edge> edges;

    if(!selected_Edges.empty()){
        edges = selected_Edges;
    }
    else{
        if(selected_Faces.size() >= 2){
            auto edge = CAD_Utility::getFaceIntersectionResult(selected_Faces[0],
                                                               selected_Faces[1]);
            edges.push_back(edge);
        }
    }

    // instanciate a edge cluster sorter and get the result
    Part::Edgecluster acluster(edges);
    Part::tEdgeClusterVector aclusteroutput = acluster.GetClusters();
    if(aclusteroutput.size() == 0){
        Base::Console().Warning("EdgebasedTracObject: Edgecluster Found No Edges specified\n");
        return calculatedPoses;
    }

    // set the number of cluster and edges
    NbrOfCluster = aclusteroutput.size();
    NbrOfEdges = 0;
    for(std::vector<std::vector<TopoDS_Edge> >::const_iterator it=aclusteroutput.begin();it!=aclusteroutput.end();++it)
        NbrOfEdges += it->size();

    // cycle trough the cluster
    for(std::vector<std::vector<TopoDS_Edge>>::const_iterator it=aclusteroutput.begin();it!=aclusteroutput.end();++it)
    {
        // cycle through the edges of the cluster
        for(std::vector<TopoDS_Edge>::const_iterator it2=it->begin();it2!=it->end();++it2)
        {
           auto newEdgePoses = CAD_Utility::generateEdgeKeyPnts(*it2,
                                                                interpoLen,
                                                                selected_Faces[0],
                                                                selected_Faces[1],
                                                                Reverse_RefFace_1.getValue(),
                                                                Reverse_RefFace_2.getValue(),
                                                                Reverse_EdgeDir.getValue());
           auto originSize = calculatedPoses.size();
           calculatedPoses.resize(originSize+newEdgePoses.size());
           std::copy(newEdgePoses.begin(),newEdgePoses.end(),calculatedPoses.begin()+originSize);
        }
    }

    return CAD_Utility::removeColinearPoints(calculatedPoses);
}
