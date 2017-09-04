#pragma once
#include "..\frame\frame.h"
#include <Eigen/Eigen/StdVector>

namespace Eigen{
	typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Matrix4dList;
	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dList;
}
namespace rbt{
	class Robot
	{
	public:
		Robot();
		Robot(const Eigen::Matrix4d& T);
		~Robot();
		void addBasicFrame(const Eigen::Matrix4d& T, int parentId = -1);
		void addRevoluteFrame(double a, double alpha, double d, double theta, double min, double max, int parentId = -1);
		void addPassiveFrame(double a, double alpha, double d, double theta, double ratio, int activeParentId, int parentId = -1);
		void seatchEE();
		void drawRobot() const;
		unsigned DOFsize() const;
		unsigned EEsize() const;
		const DHFrame& operator[] (unsigned i) const;
		DHFrame& operator[] (unsigned i);
		void getFK(std::vector<Eigen::Vector3d>& endEffectorPos) const;
		void getFK(Eigen::Matrix4dList& endEffectorMat) const;
		void getJacobian(Eigen::MatrixXd& J) const;
		void getJacobianWO(Eigen::MatrixXd& J) const;
		void reset();
		void dlsIK(const std::vector<Eigen::Vector3d>& targetPos, std::vector<double>& jointValue);
		void dlsIK(const Eigen::Matrix4dList& targetPosMat, std::vector<double>& jointValue);
	private:
		std::vector<Frame*> _allFrameList, _baseFrameList;
		std::vector<DHFrame*> _jointFrameList;
		std::vector<PDHFrame*> _passiveFrameList;
		std::vector<EndEffector*> _endEffectorList;
	};

	void transMat2Vec(const Eigen::Matrix4d& transMat, Eigen::VectorXd& P);
}