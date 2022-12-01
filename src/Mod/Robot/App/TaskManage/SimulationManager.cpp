// Created by Yixiao 2022/05/10
#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "SimulationManager.h"
#include <Base/Console.h>
#include <Base/Reader.h>
#include <Base/Writer.h>

using namespace Robot;

SimulationManager::SimulationManager() {}

//void SimulationManager::initSimulationProcess(RD_Action *t_Action) {
//  m_simType = SimulationType::Action;
//  m_Action = t_Action;
//}

//void SimulationManager::initSimulationProcess(RD_Moment *t_Moment) {
//  m_simType = SimulationType::Moment;
//  m_Moment = t_Moment;
//}

//void SimulationManager::initSimulationProcess(RD_WorkFlow *t_WorkFlow) {
//  m_simType = SimulationType::WorkFlow;
//  m_WorkFlow = t_WorkFlow;
//}

//void SimulationManager::initSimulationProcess(
//    RD_TrajectoryObject *targetTrac) {
//  m_RoboTrac = targetTrac;
//  m_simType = SimulationType::RoboTrac;
//}

void SimulationManager::changeSimStep(double new_simualtionStepLen) {
  if (new_simualtionStepLen < 2)
    m_SimulationTimeStep = new_simualtionStepLen / SimulationFrequence;
  else
    m_SimulationTimeStep = 5 * new_simualtionStepLen / SimulationFrequence;
}

void SimulationManager::changeSimObjectVisiblity(bool visible) {
//    if(m_Moment!=nullptr){
//        m_Moment->changeVisibility(visible);
//    }
}

bool SimulationManager::calculateSimulationTime() {
  m_SimulationTotalTime = 0.0;
//  switch (m_simType) {
//  case Action:
//    if (m_Action == nullptr)
//      return false;
//    m_SimulationTotalTime = m_Action->getActionDuration();
//    break;
//  case Moment:
//    if (m_Moment == nullptr)
//      return false;
//    m_SimulationTotalTime = m_Moment->getMomentDuration();
//    break;
//  case WorkFlow:
//    if (m_WorkFlow == nullptr)
//      return false;
//    m_SimulationTotalTime = m_WorkFlow->getWorkFlowDuration();
//    break;
//  case RoboTrac:
//    assert(m_RoboTrac != nullptr);
//    m_SimulationTotalTime = m_RoboTrac->getActionDuration();
//    break;
//  default:
//    return false;
//  }
  return true;
}

double SimulationManager::getSimulationTotalTime() {
  return m_SimulationTotalTime;
}

void SimulationManager::Save(Base::Writer &writer) const {}

void SimulationManager::Restore(Base::XMLReader &reader) {}

void SimulationManager::updateSimulationTargetStatus(
    const float timerPosition) {
  switch (m_simType) {
  case SimulationType::Action:
    executeActionAtTime(timerPosition);
    break;
  case SimulationType::Moment:
    executeMomentAtTime(timerPosition);
    break;
  case SimulationType::WorkFlow:
    executeWorkFlowAtTime(timerPosition);
    break;
    //  case SimulationType::RoboTrac:
    //    executeTrajectoryAtTime(timerPosition);
    break;
  default:
    break;
  }
}

bool SimulationManager::executeActionAtTime(const float t_Position) {
//  if (m_Action == nullptr)
//    return false;
//  if (m_Action->isDerivedFrom(
//          RD_TaskManager::RD_Action_LinearMovement::getClassTypeId())) {
//    auto linearMovement =
//        dynamic_cast<RD_TaskManager::RD_Action_LinearMovement *>(m_Action);
//    linearMovement->executeAction(t_Position);
//  }
  return true;
}

// bool RD_SimulationManager::executeTrajectoryAtTime(const float t_Position) {
//  if (m_RoboTrac == nullptr)
//    return false;
//  if (m_RoboTrac->executeAction(t_Position))
//    return true;
//  return false;
//}

bool SimulationManager::executeMomentAtTime(const float t_Position) {
//  if (m_Moment == nullptr)
//    return false;
//  auto executeTargetVec = m_Moment->getExecutObjAtTime(t_Position);
//  for (auto i_ActionTarget : executeTargetVec) {
//    auto actType = i_ActionTarget->getActionType();
//    switch (actType) {
//    case ActionType::Grab:
//    case ActionType::Release: {
//      auto graspAct =
//          dynamic_cast<RD_TaskManager::RD_Action_Grasping *>(i_ActionTarget);
//      graspAct->executeAction(t_Position);
//    } break;
//    case ActionType::LinearMotion: {
//      auto linearAct = dynamic_cast<RD_TaskManager::RD_Action_LinearMovement *>(
//          i_ActionTarget);
//      linearAct->executeAction(t_Position);
//    } break;
//    case ActionType::RobotTrac: {
//      auto tracAct =
//          dynamic_cast<RD_TaskManager::RD_TrajectoryObject *>(i_ActionTarget);
//      tracAct->executeAction(t_Position);
//    } break;

//    default:
//      return false;
//    }
//  }
  return true;
}

bool SimulationManager::executeWorkFlowAtTime(const float t_Position) {
//  if (m_WorkFlow == nullptr)
//    return false;
//  auto momentVec = m_WorkFlow->getMoments();
//  auto momentNum = momentVec.size();
//  double current_period = 0.0;
//  for (int i = 0; i < momentNum; i++) {
//    current_period += momentVec.at(i)->getMomentDuration();
//    if (t_Position <= current_period) {
//      m_Moment = momentVec.at(i);
//      auto executeTime = t_Position;
//      if (i > 0) {
//        auto formerDuration = 0.0;
//        for (int j = 0; j < i; j++) {
//          formerDuration += momentVec.at(j)->getMomentDuration();
//        }
//        executeTime -= formerDuration;
//      }
//      executeMomentAtTime(executeTime);
//      break;
//    }
//  }
  return true;
}
