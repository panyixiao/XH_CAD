#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <fcntl.h>
#include <BRep_Builder.hxx>
//#include <Handle_TColStd_HSequenceOfTransient.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <Message_MsgFile.hxx>
#include <TColStd_HSequenceOfTransient.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <IGESBasic_Group.hxx>
#include <IGESBasic_SingularSubfigure.hxx>
#include <IGESSolid_ManifoldSolid.hxx>

#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StepData_StepModel.hxx>

//#include <Handle_Message_ProgressIndicator.hxx>
//#include <Handle_StepShape_ShapeRepresentation.hxx>
//#include <Handle_StepVisual_PresentationStyleByContext.hxx>
//#include <Handle_XSControl_TransferReader.hxx>
//#include <Handle_XSControl_WorkSession.hxx>

#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>

#include <Interface_EntityIterator.hxx>
#include <Quantity_Color.hxx>
#include <TCollection_ExtendedString.hxx>

#include <App/Application.h>
#include <App/Document.h>
#include <Base/Console.h>
#include <Base/Sequencer.h>

#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/App/ProgressIndicator.h>
#include <Mod/Part/App/encodeFilename.h>

#include "PartUtility.h"
//#include <g3log/rlog.h>

using namespace Part;
using namespace Robot;

App::DocumentObject *PartUtility::importCADFile(App::Document *pcDoc,
                                                const std::string &filePath,
                                                CAD_fileUsage usage,
                                                bool combined) {
  if (filePath.size()) {
    Base::FileInfo fileInfo(filePath.c_str());
    if (fileInfo.hasExtension("stp") || fileInfo.hasExtension("step")) {
      return importStepModelObject(pcDoc, filePath.c_str(), usage, combined);
    }
    else if (fileInfo.hasExtension("igs") || fileInfo.hasExtension("iges")) {
      return importIgesModelObject(pcDoc, filePath.c_str(), usage, combined);
    }
    else {
      // RLOG_LOG_PRINTF(DEBUG0, "Not implemented yet");
    }
  }
  return nullptr;
}

std::vector<TopoDS_Shape> PartUtility::importStepModel(const char *filePath, bool combined)
{
    std::vector<TopoDS_Shape> t_Result;
    STEPControl_Reader aReader;
    TopoDS_Shape aShape;

    std::string encodednamestr = Part::encodeFilename(std::string(filePath));
    const char *encodedname = encodednamestr.c_str();

    if (aReader.ReadFile((Standard_CString)encodedname) != IFSelect_RetDone) {
      throw Base::RuntimeError("Cannot open STEP file");
    }

    Handle_Message_ProgressIndicator pi = new Part::ProgressIndicator(100);
    aReader.WS()->MapReader()->SetProgress(pi);
    pi->NewScope(100, "Reading STEP file...");
    pi->Show();
    // Root transfers
    Standard_Integer nbr = aReader.NbRootsForTransfer();
    // aReader.PrintCheckTransfer (failsonly, IFSelect_ItemsByEntity);
    for (Standard_Integer n = 1; n <= nbr; n++) {
      Base::Console().Log("STEP: Transferring Root %d\n", n);
      aReader.TransferRoot(n);
    }
    pi->EndScope();

    // Collecting resulting entities
    Standard_Integer nbs = aReader.NbShapes();
    if (nbs == 0) {
      throw Base::RuntimeError("No shapes found in file ");
      return t_Result;
    }

//    auto Model = aReader.StepModel();
//    auto ws = aReader.WS();
//    auto tr = ws->TransferReader();

    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);

    for (Standard_Integer i = 1; i <= nbs; i++) {
      Base::Console().Log("STEP:   Transferring Shape %d\n", i);
      Standard_Boolean emptyComp = Standard_True;
      aShape = aReader.Shape(i);
      TopExp_Explorer ex;

      for (ex.Init(aShape, TopAbs_SOLID); ex.More(); ex.Next()) {
        if (!ex.Current().IsNull()) {
            // get the shape
            if(combined){
                builder.Add(comp, ex.Current());
                emptyComp = Standard_False;
            }
            else{
                auto aSolid = TopoDS::Solid(ex.Current());
                t_Result.push_back(aSolid);
//                std::string name = fi.fileNamePure();
//                Handle_Standard_Transient ent = tr->EntityFromShapeResult(aSolid, 3);
//                if (!ent.IsNull()) {
//                  name += ws->Model()->StringLabel(ent)->ToCString();
//                }
            }
        }
      }

      for (ex.Init(aShape, TopAbs_SHELL, TopAbs_SOLID); ex.More(); ex.Next()) {
        // get the shape

        if (!ex.Current().IsNull()) {
            if(combined){
                builder.Add(comp, ex.Current());
                emptyComp = Standard_False;
            }
            else{
                const TopoDS_Shell &aShell = TopoDS::Shell(ex.Current());
//                std::string name = fi.fileNamePure();
//                Handle_Standard_Transient ent = tr->EntityFromShapeResult(aShell, 3);
//                if (!ent.IsNull()) {
//                  name += ws->Model()->StringLabel(ent)->ToCString();
//                }
                t_Result.push_back(aShell);
            }
        }
      }

      for (ex.Init(aShape, TopAbs_FACE, TopAbs_SHELL); ex.More(); ex.Next()) {
        if (!ex.Current().IsNull()) {
          builder.Add(comp, ex.Current());
          emptyComp = Standard_False;
        }
      }
      for (ex.Init(aShape, TopAbs_WIRE, TopAbs_FACE); ex.More(); ex.Next()) {
        if (!ex.Current().IsNull()) {
          builder.Add(comp, ex.Current());
          emptyComp = Standard_False;
        }
      }
      for (ex.Init(aShape, TopAbs_EDGE, TopAbs_WIRE); ex.More(); ex.Next()) {
        if (!ex.Current().IsNull()) {
          builder.Add(comp, ex.Current());
          emptyComp = Standard_False;
        }
      }
      for (ex.Init(aShape, TopAbs_VERTEX, TopAbs_EDGE); ex.More(); ex.Next()) {
        if (!ex.Current().IsNull()) {
          builder.Add(comp, ex.Current());
          emptyComp = Standard_False;
        }
      }
    }

    t_Result.push_back(comp);

    return t_Result;
}

std::vector<TopoDS_Shape> PartUtility::importIgesModel(const char *filePath, bool combined)
{
    std::vector<TopoDS_Shape> t_Result;

    Base::FileInfo fi(filePath);
    // read iges fileu
    IGESControl_Controller::Init();
    // load data exchange message files
    Message_MsgFile::LoadFromEnv("CSF_XSMessage", "IGES");
    // load shape healing message files
    Message_MsgFile::LoadFromEnv("CSF_SHMessageStd", "SHAPEStd");
    IGESControl_Reader aReader;
    if (aReader.ReadFile((const Standard_CString)filePath) != IFSelect_RetDone)
        throw Base::RuntimeError("Error in reading IGES");

    // Ignore construction elements
    // http://www.opencascade.org/org/forum/thread_20603/?forum=3
    aReader.SetReadVisible(Standard_True);
    // check file conformity and output stats
    aReader.PrintCheckLoad(Standard_True, IFSelect_GeneralInfo);
    std::string aName = fi.fileNamePure();
    Handle_Message_ProgressIndicator pi = new Part::ProgressIndicator(100);
    pi->NewScope(100, "Reading IGES file...");
    pi->Show();
    aReader.WS()->MapReader()->SetProgress(pi);
    // make model
    aReader.ClearShapes();
    // Standard_Integer nbRootsForTransfer = aReader.NbRootsForTransfer();
    aReader.TransferRoots();
    pi->EndScope();
    // put all other free-flying shapes into a single compound
    Standard_Boolean emptyComp = Standard_True;
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    Standard_Integer nbShapes = aReader.NbShapes();
    for (Standard_Integer i = 1; i <= nbShapes; i++) {
      const TopoDS_Shape aShape = aReader.Shape(i);
      if (!aShape.IsNull()) {
        if (!combined) {
          if (aShape.ShapeType() == TopAbs_SOLID ||
              aShape.ShapeType() == TopAbs_COMPOUND ||
              aShape.ShapeType() == TopAbs_SHELL) {
              t_Result.push_back(aShape);
//            insertPlanningObjToDoc(pcDoc, aShape, filePath, aName);
          }
          else {
            builder.Add(comp, aShape);
            emptyComp = Standard_False;
          }
        } else {
          // Get all parts into one.
          builder.Add(comp, aShape);
          emptyComp = Standard_False;
        }
      }
    }
    if(!emptyComp && combined)
        t_Result.push_back(comp);
    return t_Result;
}

App::DocumentObject *PartUtility::importStepModelObject(App::Document *pcDoc,
                                                 const char *filePath,
                                                 CAD_fileUsage usage,
                                                 bool combined) {

    Base::FileInfo fi(filePath);
    if (!fi.exists()) {
      std::stringstream str;
      str << "File '" << filePath << "' does not exist!";
      throw Base::RuntimeError(str.str().c_str());
      return nullptr;
    }

    App::DocumentObject *result = nullptr;

    auto shapeVec = importStepModel(filePath,combined);
    bool first = true;
    for(const auto& shape : shapeVec){
        if(!shape.IsNull()){
            std::string name = fi.fileNamePure();
            if (usage == As_PlanningOBJ) {
              auto newPlanningObj = insertPlanningObjToDoc(pcDoc, shape, filePath, name);
              if(first){
                  result = newPlanningObj;
                  first = false;
              }
            }
            else if (usage == As_ToolOBJ) {
              return insertToolObjToDoc(pcDoc, shape, filePath, name);
            }
        }
    }

    return result;
}

App::DocumentObject *PartUtility::importIgesModelObject(App::Document *pcDoc,
                                                 const char *filePath,
                                                 CAD_fileUsage usage,
                                                 bool combined) {
    Base::FileInfo fi(filePath);
    if (!fi.exists()) {
      std::stringstream str;
      str << "File '" << filePath << "' does not exist!";
      throw Base::RuntimeError(str.str().c_str());
      return nullptr;
    }

  try {
        auto result = importIgesModel(filePath,combined);

        for(const auto& shape : result){
            std::string name = fi.fileNamePure();
            if (usage == As_PlanningOBJ) {
              return insertPlanningObjToDoc(pcDoc, shape, filePath, name);
            }
            else if (usage == As_ToolOBJ) {
              return insertToolObjToDoc(pcDoc, shape, filePath, name);
            }
        }


  } catch (Standard_Failure) {
    Handle(Standard_Failure) aFail = Standard_Failure::Caught();
    throw Base::RuntimeError(aFail->GetMessageString());
  }
  return nullptr;
}

App::DocumentObject *PartUtility::insertPlanningObjToDoc(App::Document *pcDoc,
                                                         const TopoDS_Shape &shape,
                                                         const std::string &filePath,
                                                         const std::string &obj_Name)
{
  auto n_planningObj = static_cast<Robot::PlanningObject *>(pcDoc->addObject("Robot::PlanningObject", obj_Name.c_str()));
  if (n_planningObj == nullptr)
    return nullptr;
  n_planningObj->Shape.setValue(shape);
  n_planningObj->FilePath_Solid.setValue(filePath);
//  n_planningObj->ObjectName.setValue(n_planningObj->getNameInDocument());
  double mat[4][4];
  for (int i = 1; i < 4; ++i) {
    for (int j = 1; j <= 4; ++j) {
      mat[i - 1][j - 1] = shape.Location().Transformation().Value(i, j);
    }
  }
  // This is a trick to access the GUI via Python and set the
  // color property of the associated xview provider. If no GUI is up an
  // exception
  // is thrown and cleared immediately
  std::map<int, Quantity_Color> hash_col;
  std::map<int, Quantity_Color>::iterator it = hash_col.find(shape.HashCode(INT_MAX));
  if (it != hash_col.end()) {
    try {
      Py::Object obj(n_planningObj->getPyObject(), true);
      Py::Object vp(obj.getAttr("ViewObject"));
      Py::Tuple col(3);
      col.setItem(0, Py::Float(it->second.Red()));
      col.setItem(1, Py::Float(it->second.Green()));
      col.setItem(2, Py::Float(it->second.Blue()));
      vp.setAttr("ShapeColor", col);
    }
      catch (Py::Exception &e) {
      e.clear();
    }
  }
  return n_planningObj;
}

App::DocumentObject * PartUtility::insertToolObjToDoc(App::Document *pcDoc,
                                                      const TopoDS_Shape &shape,
                                                      const std::string &filePath,
                                                      const std::string &obj_Name) {
  auto result = pcDoc->addObject("Robot::ToolObject", obj_Name.c_str());
  if(result == nullptr)
      return result;
  auto toolOBJ = static_cast<Robot::ToolObject *>(result);
  (*toolOBJ).Shape.setValue(shape);
//  toolOBJ->Shape.setValue(shape);
//  toolOBJ->CAD_File.setValue(filePath);
//  double mat[4][4];
//  for (int i = 1; i < 4; ++i) {
//    for (int j = 1; j <= 4; ++j) {
//      mat[i - 1][j - 1] = shape.Location().Transformation().Value(i, j);
//    }
//  }
  return toolOBJ;
}
