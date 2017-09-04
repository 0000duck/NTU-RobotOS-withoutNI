#pragma once
#include "glut.h"
#include "Eigen\Eigen\Dense"

namespace obj{
	class BoundingBox{
	public:
		BoundingBox(): _center(Eigen::Vector3d::Zero()), _min(Eigen::Vector3d::Zero()), _max(Eigen::Vector3d::Zero()){}
		~BoundingBox(){}
		void draw() const;
		Eigen::Vector3d _center, _min, _max;
	};

	void Draw_Cute_Axis(float LINK_LENGTH);
}