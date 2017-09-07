#pragma once
#ifdef use_namespace
namespace ROBOOP {
	using namespace NEWMAT;
#endif
#include "global_value.h"
// ------------  robot library ----------
#include "roboop/robot.h"
#include "roboop/utils.h"
#include "OpenGLControl.h"
#include <thread>
extern 	Robot MyRobot;
class MyControlProject
{
	friend class Robot_basic;
	friend class Robot;
	friend class Link;
	friend class COpenGLControl;
public:

	MyControlProject();
	~MyControlProject();
	void Inital_Atom();
	void MyTorqueControl();
	void Thead1Open();
	thread *mThread;
};

