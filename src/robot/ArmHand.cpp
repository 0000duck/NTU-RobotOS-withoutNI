
#include "ArmHand.h"
#include <iostream>
using namespace rbt;


rbt::NTURobot::NTURobot(){
	const double pi = acos(-1.0);
	double d[6] = { 0, 0, 371, 0, 280, 0 }; // d[0] = 1220 平台 沒加上去
	double a[6] = { 0, 0, 10, -10, 0, 30 };
	double alpha[6] = { pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2 };
	double theta[6] = { 0, pi, pi, -pi / 2, pi, pi / 2 };
	double maxlimits[6] = { 5 * pi / 9, pi / 2, 115 * pi / 180, pi / 2, 115 * pi / 180, 85 * pi / 180 };
	double minlimits[6] = { -5 * pi / 9, -pi / 2, -115 * pi / 180, -40 * pi / 180, -115 * pi / 180, -85 * pi / 180 };
	for (int i = 0; i < 6; ++i){
		armHand.addRevoluteFrame(a[i], alpha[i], d[i], theta[i], minlimits[i], maxlimits[i], i);
	} // id1 ~ 6

	Eigen::Matrix4d temp;
	// 把end effector 座標系轉成 手掌base 座標系
	temp << 0, 0, 1, 0,
		0, 1, 0, 0,
		-1, 0, 0, 0,
		0, 0, 0, 1;
	armHand.addBasicFrame(temp, 6); // id7

	// Thumb;
	temp << 1, 0, 0, 3.4,
		0, 1, 0, -22.38,
		0, 0, 1, 52.24,
		0, 0, 0, 1;
	armHand.addBasicFrame(temp, 7); // id8
	armHand.addRevoluteFrame(50.74, pi / 2, 13.08, -pi / 2, 0, pi / 2, 8); // id9
	armHand.addRevoluteFrame(38.95, 0, 0, pi / 4, 0, pi / 2, 9); // id10
	armHand.addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 10); // id11
	armHand.addPassiveFrame(21.6, 0, 0, 0, 0.82, 11, 11); // id12

	// Index
	temp << 0, 0, 1, 12.05,
		1, 0, 0, -42.25,
		0, 1, 0, 80.59,
		0, 0, 0, 1;
	armHand.addBasicFrame(temp, 7); // id13
	armHand.addRevoluteFrame(44.5, pi / 2, -9.65, pi / 2, 0, pi / 12, 13); // id14
	armHand.addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 14); // id15
	armHand.addRevoluteFrame(25.92, 0, 0, 0, 0, pi * 99 / 180, 15); // id16
	armHand.addPassiveFrame(21.6, 0, 0, 0, 0.82, 16, 16); // id17

	// Middle
	temp << 0, 1, 0, 2.4,
		0, 0, 1, -17.4,
		1, 0, 0, 138.22,
		0, 0, 0, 1;
	armHand.addBasicFrame(temp, 7); // id18
	armHand.addRevoluteFrame(38.88, 0, 0, 0, 0, pi / 2, 18); // id19
	armHand.addRevoluteFrame(25.92, 0, 0, 0, 0, pi * 99 / 180, 19); // id20
	armHand.addPassiveFrame(21.6, 0, 0, 0, 0.82, 20, 20); // id21

	// Ring
	temp << 0, 0, -1, 12.05,
		-1, 0, 0, 5.75,
		0, 1, 0, 80.59,
		0, 0, 0, 1;
	armHand.addBasicFrame(temp, 7); // id22
	armHand.addPassiveFrame(44.5, -pi / 2, 9.65, pi / 2, 1, 14, 22); // id23
	armHand.addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 23); // id24
	armHand.addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 24); // id25
	armHand.addPassiveFrame(21.6, 0, 0, 0, 0.82, 25, 25); // id26

	// Pinky
	temp << 0, 0, -1, 12.05,
		-1, 0, 0, 29.75,
		0, 1, 0, 80.59,
		0, 0, 0, 1;
	armHand.addBasicFrame(temp, 7); // id27
	armHand.addPassiveFrame(44.5, -pi / 2, 9.65, pi / 2, 21.5 / 15, 14, 27); // id28
	armHand.addRevoluteFrame(38.9, 0, 0, 0, 0, pi / 2, 28); // id29
	armHand.addRevoluteFrame(25.92, 0, 0, 0, 0, 99 * pi / 180, 29); // id30
	armHand.addPassiveFrame(21.6, 0, 0, 0, 0.82, 30, 30); // id31

	armHand.seatchEE();

	a[5] = 100;
	for (int i = 0; i < 6; ++i){
		arm.addRevoluteFrame(a[i], alpha[i], d[i], theta[i], minlimits[i], maxlimits[i], i);
	}
	arm.seatchEE();
}

rbt::NTURobot::~NTURobot(){}

void rbt::NTURobot::drawRobot() const  {
	armHand.drawRobot();
}

void rbt::NTURobot::armFK(Eigen::Vector3d& endEffectorPos) {
	updateArm();
	std::vector<Eigen::Vector3d> computePos;
	arm.getFK(computePos);
	endEffectorPos = computePos.front();
}

void rbt::NTURobot::armFK(Eigen::Matrix4d& endEffectorMat) {
	updateArm();
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> computeMat;
	arm.getFK(computeMat);
	endEffectorMat = computeMat.front();
}

void rbt::NTURobot::handFK(std::vector<Eigen::Vector3d>& endEffectorPos) const{
	armHand.getFK(endEffectorPos);
}

void rbt::NTURobot::handFK(Eigen::Matrix4dList& endEffectirMat) const{
	armHand.getFK(endEffectirMat);
}

void rbt::NTURobot::getArmJacobian(Eigen::MatrixXd& jacobian){
	updateArm();
	arm.getJacobian(jacobian);
}

void rbt::NTURobot::getArmJacobianWO(Eigen::MatrixXd& jacobian){
	updateArm();
	arm.getJacobianWO(jacobian);
}

void rbt::NTURobot::getHandJacobian(Eigen::MatrixXd& jacobian) const{
	armHand.getJacobian(jacobian);
}

void rbt::NTURobot::getHandJacobianWO(Eigen::MatrixXd& jacobian) const{
	armHand.getJacobianWO(jacobian);
}


const DHFrame& rbt::NTURobot::operator[] (unsigned i) const{
	return armHand[i];
}

DHFrame& rbt::NTURobot::operator[] (unsigned i){
	return armHand[i];
}

unsigned rbt::NTURobot::dof() const{
	return armHand.DOFsize();
}

void rbt::NTURobot::armIK(const Eigen::Vector3d& targetPos, std::vector<double>& jointValue){
	std::vector<Eigen::Vector3d> target;
	target.push_back(targetPos);
	arm.dlsIK(target, jointValue);
}

void rbt::NTURobot::armIK(const Eigen::Matrix4d& targetPosMat, std::vector<double>& jointValue){
	Eigen::Matrix4dList target;
	target.push_back(targetPosMat);
	arm.dlsIK(target, jointValue);
}

void rbt::NTURobot::handIK(const std::vector<Eigen::Vector3d>& targetPos, std::vector<double>& jointValue){
	armHand.dlsIK(targetPos, jointValue);
}

void rbt::NTURobot::handIK(const Eigen::Matrix4dList& targetPosMat, std::vector<double>& jointValue){
	armHand.dlsIK(targetPosMat, jointValue);
}

void rbt::NTURobot::sendJointValue(const std::vector<double>& jointValue){
	if (jointValue.size() <= dof()){
		for (int i = 0, n = jointValue.size(); i < n; ++i){
			armHand[i].setCmd(jointValue[i]);
			if (i < 6){
				arm[i].setCmd(jointValue[i]);
			}
		}
	}
	else{
		std::cout << "DOF error!" << std::endl;
	}
}

void rbt::NTURobot::getJointValue(std::vector<double>& jointValue){
	jointValue.clear();
	for (unsigned i = 0, n = dof(); i < n; ++i){
		jointValue.push_back(armHand[i].getCmd());
	}
}

void rbt::NTURobot::updateArm(){
	for (unsigned i = 0, n = arm.DOFsize(); i < n; ++i){
		arm[i].setCmd(armHand[i].getCmd());
	}
}

void rbt::NTURobot::reset(){
	for (unsigned i = 0, n = dof(); i < n; ++i){
		armHand[i].setCmd(0);
	}
	updateArm();
}