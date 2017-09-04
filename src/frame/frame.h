#pragma once
#include "Eigen/Eigen/Dense"
#include <Eigen/Eigen/StdVector>
#include "glut.h"
#include <math.h>
#include <vector>

namespace rbt{
	class Frame;
	class BasicFrame;
	class PDHFrame;
	class DHFrame;
	class EndEffector;

	class Frame
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // for eigen fix matrix
		Frame();
		virtual ~Frame();

		virtual void draw() const = 0;
		void addChild(Frame* child);
		void getFK(std::vector<Eigen::Vector3d>& endEffectorPos, const Eigen::Matrix4d& T) const;
		void getFK(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& endEffectorMat, const Eigen::Matrix4d& T) const;
		void searchEE(std::vector<Frame*> path, std::vector<EndEffector*>& list);
		const Eigen::Matrix4d& getTransMat();
		virtual bool isDrive();
		virtual bool isPassive();
	protected:
		Eigen::Matrix4d _transMat;
		std::vector<Frame*> _children;
	};

	class BasicFrame :public Frame
	{
	public:
		BasicFrame();
		BasicFrame(const Eigen::Matrix4d& T);
		~BasicFrame();

		void draw() const;
	};


	class PDHFrame :public Frame // Passive DH Frame
	{
	public:
		PDHFrame();
		PDHFrame(double a, double alpha, double d, double theta, double ratio, DHFrame* activeParent);
		~PDHFrame();

		void draw() const;
		unsigned getDriveId();
		double getRatio();
		bool isPassive();
		void updateTransMat();
	private:
		double _a, _alpha, _d, _theta;
		double _ratio;
		double* _parentCmd;
		DHFrame* _activeParent;
	};

	class DHFrame :public Frame
	{
	public:
		DHFrame();
		DHFrame(double a, double alpha, double d, double theta, double min, double max);
		~DHFrame();

		void setCmd(double cmd);
		void updateCmd(double cmd);
		double getCmd();
		void setId(unsigned id);
		unsigned getId();
		void draw() const;
		bool isDrive();
		double* getCmdAddress();
		void addPassiveChild(PDHFrame* child);
		void getCmdRange(double& min, double& max);
	private:
		double _a, _alpha, _d, _theta;
		double _max, _min, _cmd;
		unsigned _id;
		std::vector<PDHFrame*> _passiveChildren;
		void updateTransMat();
	};

	class EndEffector
	{
	public:
		EndEffector();
		EndEffector(const std::vector<Frame*>& p);
		~EndEffector();

		void addPath(const std::vector<Frame*>& p);
		const std::vector<Frame*>& getPath();
	private:
		std::vector<Frame*> _path;
	};

	void Draw_Cute_Axis(float LINK_LENGTH);  //µe¶b ¹êÅç«Ç³Ð
	void drawCylinder(double Radius, double Height, float red = 1, float green = 0, float blue = 0);
	void drawRect(double Length, double Width, double Height, float red = 1, float green = 0, float blue = 0);
}