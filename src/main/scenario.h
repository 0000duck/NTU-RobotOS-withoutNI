#pragma once
#include "..\robot\ArmHand.h"
#include "..\object\object.h"
#include "..\trajPlan\trajPlan.h"

extern rbt::NTURobot ntuRobot;
extern obj::BoundingBox box;
extern obj::BoundingBox gotoBox;
extern std::vector<Eigen::MatrixXd> path;
extern Eigen::Vector3d armLift, armDown;
extern bool motionFlag;

void loadBoundingBox(char* filename);
void scenarioShow();
void graspPlanning();
void releasePlanning();
void pathPrintOut();
void swPrintOut();