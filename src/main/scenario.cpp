
#include "scenario.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "glut.h"
#include <cmath>

static double pi = acos(-1.0);

rbt::NTURobot ntuRobot;
obj::BoundingBox box;
obj::BoundingBox gotoBox;
// for grasping
Eigen::Matrix4dList armViaPoint;
std::vector<Eigen::Matrix4dList> handViaPoint;
std::vector<Eigen::MatrixXd> path;
// for release planning
Eigen::Matrix4d armGraspingPosMat; // 第一個規劃arm的T 需要拿orientation
Eigen::Vector3d relativePos; // box質心跟arm endeffector的差 用來估arm的位置用
std::vector<double> fingerGraspingJointValue;
// for movie
bool motionFlag = false;
// for arm
Eigen::Vector3d armLift = Eigen::Vector3d::Zero(), armDown = Eigen::Vector3d::Zero();

void loadBoundingBox(char* filename){
	std::fstream fin;
	fin.open(filename, std::ios::in);
	Eigen::Vector3d vec;
	for (int i = 0; i < 3; ++i){
		fin >> vec(i);
	}
	box._center = vec;
	for (int i = 0; i < 3; ++i){
		fin >> vec(i);
	}
	box._min = vec;
	for (int i = 0; i < 3; ++i){
		fin >> vec(i);
	}
	box._max = vec;

	for(int i = 0; i < 3; ++i){
		fin >> vec(i);
	}
	vec -= box._center;
	box._center += vec;
	box._max += vec;
	box._min += vec;
	gotoBox = box;

	for(int i = 0; i < 3; ++i){
		fin >> vec(i);
	}
	vec -= gotoBox._center;
	gotoBox._center += vec;
	gotoBox._max += vec;
	gotoBox._min += vec;
	fin.close();
}

void scenarioShow(){
	glTranslatef(-300, 0, -180);
	ntuRobot.drawRobot();
	box.draw();
	for (int i = 0, n = armViaPoint.size(); i < n; ++i){
		glPushMatrix();
		glMultMatrixd(armViaPoint[i].data());
		rbt::Draw_Cute_Axis(15);
		glColor3f(0.7, 0.5, 0);
		glutWireSphere(3, 10, 10);
		glPopMatrix();
	}
	for (int i = 0, n = handViaPoint.size(); i < n; ++i){
		for (int j = 0, m = handViaPoint[i].size(); j < m; ++j){
			glPushMatrix();
			glMultMatrixd(handViaPoint[i][j].data());
			rbt::Draw_Cute_Axis(15);
			glColor3f(1-0.2*j, 0, 0.2*j);
			glutWireSphere(3, 10, 10);
			glPopMatrix();
		}
	}
}

void graspPlanning(){

	const double &x1 = box._min(0), &y1 = box._min(1), &z1 = box._min(2);
	const double &x2 = box._max(0), &y2 = box._max(1), &z2 = box._max(2);
	const double delx = x2 - x1, dely = y2 - y1, delz = z2 - z1;
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	Eigen::Matrix4dList finger;
	Eigen::Matrix3d R;
	Eigen::Vector3d p;

	R = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
	T.block<3, 3>(0, 0) = R;
	T(0, 3) = x1 - 80;
	T(1, 3) = box._center(1);
	T(2, 3) = z2 + 120;
	armViaPoint.push_back(T);
	finger.push_back(Eigen::Matrix4d::Identity());
	finger.push_back(Eigen::Matrix4d::Identity());
	finger.push_back(Eigen::Matrix4d::Identity());
	R << 0, 1, 0,
		0, 0, -1,
		-1, 0, 0;
	p << x1 - 30, box._center(1), box._center(2) + 18;
	finger[0].block<3, 1>(0, 3) = p;
	finger[0].block<3, 3>(0, 0) = R;
	R << 0, -1, 0,
		0, 0, 1,
		-1, 0, 0;
	p << x2 + 30, box._center(1) - 12.5, box._center(2) + 18;
	finger[1].block<3, 1>(0, 3) = p;
	finger[1].block<3, 3>(0, 0) = R;
	R << 0, -1, 0,
		0, 0, 1,
		-1, 0, 0;
	p << x2 + 30, box._center(1) + 12.5, box._center(2) + 10;
	finger[2].block<3, 1>(0, 3) = p;
	finger[2].block<3, 3>(0, 0) = R;

	handViaPoint.push_back(finger);
	R << 0, 1, 0,
		0, 0, -1,
		-1, 0, 0;
	p << x1, box._center(1), box._center(2) + 4;
	finger[0].block<3, 1>(0, 3) = p;
	finger[0].block<3, 3>(0, 0) = R;
	R << 0, -1, 0,
		0, 0, 1,
		-1, 0, 0;
	p << x2, box._center(1) - 12.5, box._center(2) + 4;
	finger[1].block<3, 1>(0, 3) = p;
	finger[1].block<3, 3>(0, 0) = R;
	R << 0, -1, 0,
		0, 0, 1,
		-1, 0, 0;
	p << x2, box._center(1) + 12.5, box._center(2) - 4;
	finger[2].block<3, 1>(0, 3) = p;
	finger[2].block<3, 3>(0, 0) = R;
	handViaPoint.push_back(finger);

	// trajectory planning
	Eigen::VectorXd q0 = Eigen::VectorXd::Zero(ntuRobot.dof());
	Eigen::VectorXd q0d = Eigen::VectorXd::Zero(ntuRobot.dof());
	Eigen::VectorXd qf = Eigen::VectorXd::Zero(ntuRobot.dof());
	Eigen::VectorXd qfd = Eigen::VectorXd::Zero(ntuRobot.dof());
	std::vector<double> th, th0;
	ntuRobot.armIK(armViaPoint.back(), th0);
	th0.push_back(pi / 2);
	ntuRobot.sendJointValue(th0);
	for (int i = 0, n = th0.size(); i < n; ++i){
		q0(i) = th0[i];
	}
	ntuRobot.handIK(handViaPoint[0], th);
	for (int i = 0, n = th.size(); i < n; ++i){
		qf(i) = th[i];
	}
	double time = 5;
	path.push_back(tp::Splines212(q0,q0d,qf,qfd,time,1));
	ntuRobot.sendJointValue(th);
	q0 = qf;
	ntuRobot.handIK(handViaPoint.back(), th);

	// 壓進去些xdd
	th[7] += 3 * pi / 180;
	th[10] += 3 * pi / 180;
	th[12] += 3 * pi / 180;

	fingerGraspingJointValue = th;
	fingerGraspingJointValue.erase(fingerGraspingJointValue.begin(), fingerGraspingJointValue.begin() + 6);
	ntuRobot.sendJointValue(th);
	Eigen::Vector3d& armPos = relativePos;
	ntuRobot.armFK(armGraspingPosMat);
	armPos = armGraspingPosMat.block<3, 1>(0, 3);
	relativePos = armPos - box._center;
	for (int i = 0, n = th.size(); i < n; ++i){
		qf(i) = th[i];
	}
	path.push_back(tp::Splines212(q0, q0d, qf, qfd, time, 1));
	q0 = qf;
	for (int i = 0, n = th0.size(); i < n; ++i){
		qf(i) = th0[i];
	}
	path.push_back(tp::Splines212(q0, q0d, qf, qfd, time, 1));
	ntuRobot.reset();
	ntuRobot.sendJointValue(th0);
}

void releasePlanning(){

	Eigen::Matrix4d armT;
	std::vector<double> th;
	armT = armGraspingPosMat;
	armT.block<3, 1>(0, 3) = gotoBox._center + relativePos;
	ntuRobot.armIK(armT, th);
	for (int i = 0, n = fingerGraspingJointValue.size(); i < n; ++i){
		th.push_back(fingerGraspingJointValue[i]);
	}

	// trajectory planning
	Eigen::VectorXd q0 = Eigen::VectorXd::Zero(ntuRobot.dof());
	Eigen::VectorXd q0d = Eigen::VectorXd::Zero(ntuRobot.dof());
	Eigen::VectorXd qf = Eigen::VectorXd::Zero(ntuRobot.dof());
	Eigen::VectorXd qfd = Eigen::VectorXd::Zero(ntuRobot.dof());

	for (int i = 0, n = th.size(); i < n; ++i){
		q0(i) = th[i];
		if (i < 7){
			qf(i) = th[i];
		}
	}
	double time = 5;
	path.push_back(tp::Splines212(q0, q0d, qf, qfd, time, 1));
}

void swPrintOut(){
	std::fstream fout;
	std::ostringstream os;
	std::string filename;
	for (int i = 0, n = 16; i < n; ++i){
		os << i;
		filename = "sw/" + os.str() + ".txt";
		fout.open(filename.c_str(), std::ios::out);
		double time = 0;
		if (i == 9){
			for (int j = 0, m = path.size(); j < m; ++j){
				for (int k = 0, o = path[j].rows(); k < o; ++k){
					fout << time << "," << path[j](k, 8)*0.82*180/pi << std::endl;
					time += 0.01;
				}
			}
		}
		else if (i == 12){
			for (int j = 0, m = path.size(); j < m; ++j){
				for (int k = 0, o = path[j].rows(); k < o; ++k){
					fout << time << "," << path[j](k, 11)*0.82 * 180 / pi << std::endl;
					time += 0.01;
				}
			}
		}
		else if (i == 13){
			for (int j = 0, m = path.size(); j < m; ++j){
				for (int k = 0, o = path[j].rows(); k < o; ++k){
					fout << time << "," << path[j](k, 12) * 180 / pi << std::endl;
					time += 0.01;
				}
			}
		}
		else if (i == 14){
			for (int j = 0, m = path.size(); j < m; ++j){
				for (int k = 0, o = path[j].rows(); k < o; ++k){
					fout << time << "," << path[j](k, 13) * 180 / pi << std::endl;
					time += 0.01;
				}
			}
		}
		else if (i == 15){
			for (int j = 0, m = path.size(); j < m; ++j){
				for (int k = 0, o = path[j].rows(); k < o; ++k){
					fout << time << "," << path[j](k, 13)*0.82 * 180 / pi << std::endl;
					time += 0.01;
				}
			}
		}
		else{
			for (int j = 0, m = path.size(); j < m; ++j){
				for (int k = 0, o = path[j].rows(); k < o; ++k){
					fout << time << "," << path[j](k, i) * 180 / pi << std::endl;
					time += 0.01;
				}
			}
		}
		os.str("");
		fout.close();
	}
}

void pathPrintOut(){
	std::fstream fout[2];
	std::string filename[2];
	// atom
	filename[0] = "path/atom.txt";
	fout[0].open(filename[0].c_str(), std::ios::out);
	// hand
	filename[1] = "path/hand.txt";
	fout[1].open(filename[1].c_str(), std::ios::out);
	for (int i = 0, n = path.size()-1; i < n; ++i){
		for (int j = 0, m = path[i].rows(); j < m; ++j){
			for (int k = 0, o = path[i].cols(); k < o; ++k){
				if (k < 6){
					if (k != 5){
						fout[0] << path[i](j, k) << "\t";
					}
					else{
						fout[0] << path[i](j, k);
					}
				}
				else if (k < 9){
					fout[1] << path[i](j, k) << "\t";
				}
				else if (k > 9 && k < 14){
					if (k != 13){
						fout[1] << path[i](j, k) << "\t";
					}
					else{
						fout[1] << path[i](j, k);
					}
				}
			}
			fout[0] << std::endl;
			fout[1] << std::endl;
		}
	}

	fout[0].close();
	fout[1].close();

	// atom
	filename[0] = "path/atom2.txt";
	fout[0].open(filename[0].c_str(), std::ios::out);
	// hand
	filename[1] = "path/hand2.txt";
	fout[1].open(filename[1].c_str(), std::ios::out);
	for (int j = 0, m = path.back().rows(); j < m; ++j){
		for (int k = 0, o = path.back().cols(); k < o; ++k){
			if (k < 6){
				if (k != 5){
					fout[0] << path.back()(j, k) << "\t";
				}
				else{
					fout[0] << path.back()(j, k);
				}
			}
			else if (k < 9){
				fout[1] << path.back()(j, k) << "\t";
			}
			else if (k > 9 && k < 14){
				if (k != 13){
					fout[1] << path.back()(j, k) << "\t";
				}
				else{
					fout[1] << path.back()(j, k);
				}
			}
		}
		fout[0] << std::endl;
		fout[1] << std::endl;
	}
}