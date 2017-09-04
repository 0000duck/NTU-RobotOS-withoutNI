#pragma once
#include "robot.h"

namespace rbt{
	class NTURobot{
	public:
		NTURobot();
		~NTURobot();

		void drawRobot() const;
		void armFK(Eigen::Vector3d& endEffectorPos);
		void armFK(Eigen::Matrix4d& endEffectorMat);
		void handFK(std::vector<Eigen::Vector3d>& endEffectorPos) const;
		void handFK(Eigen::Matrix4dList& endEffectirMat) const;
		void getArmJacobian(Eigen::MatrixXd& jacobian);
		void getArmJacobianWO(Eigen::MatrixXd& jacobian);
		void getHandJacobian(Eigen::MatrixXd& jacobian) const;
		void getHandJacobianWO(Eigen::MatrixXd& jacobian) const;
		void armIK(const Eigen::Vector3d& targetPos, std::vector<double>& jointValue);
		void armIK(const Eigen::Matrix4d& targetPosMat, std::vector<double>& jointValue);
		void handIK(const std::vector<Eigen::Vector3d>& targetPos, std::vector<double>& jointValue);
		void handIK(const Eigen::Matrix4dList& targetPosMat, std::vector<double>& jointValue);
		void sendJointValue(const std::vector<double>& jointValue);
		void getJointValue(std::vector<double>& jointValue);
		void reset();
		unsigned dof() const;
		const DHFrame& operator[] (unsigned i) const;
		DHFrame& operator[] (unsigned i);
	private:
		void updateArm();
		Robot armHand, arm;
	};
}