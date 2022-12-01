// Created by Yixiao 2016/10/09

#pragma once
#include <App/Document.h>
#include <Base/FileInfo.h>
#include <TopoDS_Shape.hxx>

#include "Mod/Robot/App/PlanningObj/PlanningObject.h"
#include "Mod/Robot/App/Tool/ToolObject.h"

//using namespace RD;
//using namespace RD_Setup;
namespace Robot {
enum CAD_fileUsage { As_PlanningOBJ, As_ToolOBJ };

using TopoShapeInfo = std::pair<std::string, const std::shared_ptr<TopoDS_Shape>>;
using TopoShapeContainer = std::vector<TopoShapeInfo>;
class PartUtility {
public:
  static App::DocumentObject *
  importCADFile(App::Document *pcDoc, const std::string &filePath,
                CAD_fileUsage usage = As_PlanningOBJ, bool combined = true);

  static std::vector<TopoDS_Shape> importStepModel(const char *filePath, bool combined = true);
  static std::vector<TopoDS_Shape> importIgesModel(const char *filePath, bool combined = true);

protected:
  static App::DocumentObject *
  insertPlanningObjToDoc(App::Document *pcDoc, const TopoDS_Shape &shape,
                         const std::string &filePath,
                         const std::string &obj_Name);
  static App::DocumentObject *insertToolObjToDoc(App::Document *pcDoc,
                                                 const TopoDS_Shape &shape,
                                                 const std::string &filePath,
                                                 const std::string &obj_Name);

  static App::DocumentObject *
  importStepModelObject(App::Document *pcDoc, const char *filePath,
                 CAD_fileUsage usage = As_PlanningOBJ, bool combined = true);

  static App::DocumentObject *
  importIgesModelObject(App::Document *pcDoc, const char *filePath,
                 CAD_fileUsage usage = As_PlanningOBJ, bool combined = true);
};
}
