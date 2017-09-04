
#include "frame.h"
#include "glut.h"
using namespace rbt;


//===============================================Frame=====================================
rbt::Frame::Frame(){ _transMat = Eigen::Matrix4d::Identity(); }
rbt::Frame::~Frame(){}

void rbt::Frame::addChild(Frame* child){
	_children.push_back(child);
}

void rbt::Frame::getFK(std::vector<Eigen::Vector3d>& endEffectorPos, const Eigen::Matrix4d& T) const{
	Eigen::Matrix4d newT;
	newT = T*_transMat;
	if (_children.empty()){
		endEffectorPos.push_back(newT.block<3, 1>(0, 3));
	}
	else{
		for (int i = 0, n = _children.size(); i < n; ++i)
			_children[i]->getFK(endEffectorPos, newT);
	}
}

void rbt::Frame::getFK(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& endEffectorMat, const Eigen::Matrix4d& T) const{
	Eigen::Matrix4d newT;
	newT = T*_transMat;
	if (_children.empty()){
		endEffectorMat.push_back(newT);
	}
	else{
		for (int i = 0, n = _children.size(); i < n; ++i)
			_children[i]->getFK(endEffectorMat, newT);
	}
}

void rbt::Frame::searchEE(std::vector<Frame*> path, std::vector<EndEffector*>& list){
	path.push_back(this);
	if (_children.empty()){
		list.push_back(new EndEffector(path));
	}
	else{
		for (int i = 0, n = _children.size(); i < n; ++i){
			_children[i]->searchEE(path, list);
		}
	}
}

const Eigen::Matrix4d& rbt::Frame::getTransMat(){
	return _transMat;
}

bool rbt::Frame::isDrive(){
	return false;
}

bool rbt::Frame::isPassive(){
	return false;
}
//=========================================================================================
//==========================================BasicFrame=====================================
rbt::BasicFrame::BasicFrame(){}
rbt::BasicFrame::BasicFrame(const Eigen::Matrix4d& T){ _transMat = T; }
rbt::BasicFrame::~BasicFrame(){}

void rbt::BasicFrame::draw() const{
	glMultMatrixd(_transMat.data());
	for (int i = 0, n = _children.size(); i < n; ++i){
		glPushMatrix();
		_children[i]->draw();
		glPopMatrix();
	}
}
//=========================================================================================
//============================================PDHFrame=====================================
rbt::PDHFrame::PDHFrame() {
	_a = 0; _alpha = 0; _d = 0; _theta = 0; _ratio = 0; _parentCmd = 0;
	updateTransMat();
}
rbt::PDHFrame::PDHFrame(double a, double alpha, double d, double theta, double ratio, DHFrame* activeParent){
	_a = a; _alpha = alpha; _d = d; _theta = theta; _ratio = ratio; _activeParent = activeParent;
	updateTransMat();
}
rbt::PDHFrame::~PDHFrame(){}

void rbt::PDHFrame::draw() const{
	drawCylinder(4, 15, 0, 1, 0.3);
	glMultMatrixd(_transMat.data());
	glPushMatrix();
	glRotatef(-90, 0, 1, 0);
	drawRect(15, 4, _a, 0.8, 0.3, 0);
	glPopMatrix();
	for (int i = 0, n = _children.size(); i < n; ++i){
		glPushMatrix();
		_children[i]->draw();
		glPopMatrix();
	}

	if (_children.empty())
		Draw_Cute_Axis(15);
}

unsigned rbt::PDHFrame::getDriveId(){
	return _activeParent->getId();
}

double rbt::PDHFrame::getRatio(){
	return _ratio;
}

bool rbt::PDHFrame::isPassive(){
	return true;
}

void rbt::PDHFrame::updateTransMat(){
	double _cmd = _ratio * (_activeParent->getCmd());
	_transMat(0, 0) = cos(_theta + _cmd);
	_transMat(1, 0) = sin(_theta + _cmd);
	_transMat(0, 1) = -sin(_theta + _cmd)*cos(_alpha);
	_transMat(1, 1) = cos(_theta + _cmd)*cos(_alpha);
	_transMat(2, 1) = sin(_alpha);
	_transMat(0, 2) = sin(_theta + _cmd)*sin(_alpha);
	_transMat(1, 2) = -cos(_theta + _cmd)*sin(_alpha);
	_transMat(2, 2) = cos(_alpha);
	_transMat(0, 3) = _a*cos(_theta + _cmd);
	_transMat(1, 3) = _a*sin(_theta + _cmd);
	_transMat(2, 3) = _d;
	_transMat(3, 3) = 1;
}
//=========================================================================================
//============================================DHFrame======================================
rbt::DHFrame::DHFrame(){
	_a = 0; _alpha = 0; _d = 0; _theta = 0; _min = 0; _max = 0; _cmd = 0; _id = 0;
	updateTransMat();
}
rbt::DHFrame::DHFrame(double a, double alpha, double d, double theta, double min, double max){
	_a = a; _alpha = alpha; _d = d; _theta = theta; _min = min; _max = max; _cmd = 0; _id = 0;
	updateTransMat();
}
rbt::DHFrame::~DHFrame(){}

void rbt::DHFrame::setCmd(double cmd){
	_cmd = cmd; updateTransMat();
}

void rbt::DHFrame::updateCmd(double cmd){
	_cmd += cmd; updateTransMat();
}

double rbt::DHFrame::getCmd(){
	return _cmd;
}

void rbt::DHFrame::setId(unsigned id){
	_id = id;
}

unsigned rbt::DHFrame::getId(){
	return _id;
}

void rbt::DHFrame::draw() const{
	if (_id < 6){
		drawCylinder(14, 20, 0, 1, 0.3);
		glMultMatrixd(_transMat.data());

		if (_a > _d){
			glPushMatrix();
			glRotatef(-90, 0, 1, 0);
			GLUquadricObj *p = gluNewQuadric();
			glColor3f(0.2, 0.3, 0.27);
			gluQuadricDrawStyle(p, GLU_LINE); // 畫成Wire
			gluCylinder(p, 13, 13, _a, 10, (int)_a / 14);
			glPopMatrix();
		}
		else if (_a < _d){
			glPushMatrix();
			glRotatef(90, 1, 0, 0);
			GLUquadricObj *p = gluNewQuadric();
			glColor3f(0.2, 0.3, 0.27);
			gluQuadricDrawStyle(p, GLU_LINE); // 畫成Wire
			gluCylinder(p, 13, 13, _d, 10, (int)_d / 14);
			glPopMatrix();
		}
	}
	else{ // hand
		drawCylinder(4, 15, 0, 1, 0.3);
		glMultMatrixd(_transMat.data());
		glPushMatrix();
		glRotatef(-90, 0, 1, 0);
		drawRect(15, 4, _a, 0.8, 0.3, 0);
		glPopMatrix();
	}

	for (int i = 0, n = _children.size(); i < n; ++i){
		glPushMatrix();
		_children[i]->draw();
		glPopMatrix();
	}

	if (_children.empty())
		Draw_Cute_Axis(15);
}

bool rbt::DHFrame::isDrive(){
	return true;
}

double* rbt::DHFrame::getCmdAddress(){
	return &_cmd;
}

void rbt::DHFrame::addPassiveChild(PDHFrame* child){
	_passiveChildren.push_back(child);
}

void rbt::DHFrame::getCmdRange(double& min, double& max){
	min = _min;
	max = _max;
}

void rbt::DHFrame::updateTransMat(){
	if (_cmd > _max)
		_cmd = _max;
	else if (_cmd < _min)
		_cmd = _min;
	_transMat(0, 0) = cos(_theta + _cmd);
	_transMat(1, 0) = sin(_theta + _cmd);
	_transMat(0, 1) = -sin(_theta + _cmd)*cos(_alpha);
	_transMat(1, 1) = cos(_theta + _cmd)*cos(_alpha);
	_transMat(2, 1) = sin(_alpha);
	_transMat(0, 2) = sin(_theta + _cmd)*sin(_alpha);
	_transMat(1, 2) = -cos(_theta + _cmd)*sin(_alpha);
	_transMat(2, 2) = cos(_alpha);
	_transMat(0, 3) = _a*cos(_theta + _cmd);
	_transMat(1, 3) = _a*sin(_theta + _cmd);
	_transMat(2, 3) = _d;
	_transMat(3, 3) = 1;

	for (int i = 0, n = _passiveChildren.size(); i < n; ++i){
		_passiveChildren[i]->updateTransMat();
	}
}
//=========================================================================================
//========================================EndEffector======================================
rbt::EndEffector::EndEffector(){}
rbt::EndEffector::EndEffector(const std::vector<Frame*>& p){ _path = p; }
rbt::EndEffector::~EndEffector(){}

void rbt::EndEffector::addPath(const std::vector<Frame*>& p){ _path = p; }
const std::vector<Frame*>& rbt::EndEffector::getPath(){ return _path; }
//=========================================================================================
//==================================================draw function==============================================
void rbt::Draw_Cute_Axis(float LINK_LENGTH){
	GLUquadricObj *quadratic;
	quadratic = gluNewQuadric();
	float LINK_RADIUS = 0.1;

	glEnable(GL_COLOR_MATERIAL);

	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	gluCylinder(quadratic, LINK_RADIUS, LINK_RADIUS, LINK_LENGTH, 30, 30);
	glTranslatef(0, 0, LINK_LENGTH);
	glutSolidCone(0.5, 1, 30, 30);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0);
	glRotatef(-90, 1, 0, 0);
	gluCylinder(quadratic, LINK_RADIUS, LINK_RADIUS, LINK_LENGTH, 30, 30);
	glTranslatef(0, 0, LINK_LENGTH);
	glutSolidCone(0.5, 1, 30, 30);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glRotatef(90, 0, 1, 0);
	gluCylinder(quadratic, LINK_RADIUS, LINK_RADIUS, LINK_LENGTH, 30, 30);
	glTranslatef(0, 0, LINK_LENGTH);
	glutSolidCone(0.5, 1, 150, 150);
	glPopMatrix();

	glDisable(GL_COLOR_MATERIAL);
}

void rbt::drawCylinder(double Radius, double Height, float red, float green, float blue){
	glColor3f(red, green, blue);
	static int slice = 10;
	GLUquadricObj *p = gluNewQuadric();
	gluQuadricDrawStyle(p, GLU_LINE); // 畫成Wire
	gluCylinder(p, Radius, Radius, Height / 2, slice, slice);
	glPushMatrix();
	glTranslatef(0, 0, Height / 2);
	gluDisk(p, 0, Radius, slice, slice);
	glPopMatrix();
	glPushMatrix();
	glRotatef(180, 1.0, 0.0, 0.0);
	gluCylinder(p, Radius, Radius, Height / 2, slice, slice);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(0, 0, -Height / 2);
	gluDisk(p, 0, Radius, slice, slice);
	glPopMatrix();
}

void rbt::drawRect(double Length, double Width, double Height, float red, float green, float blue){
	glColor3f(red, green, blue);
	glPushMatrix();
	glTranslatef(0, 0, Height / 2);
	glScaled(Length, Width, Height);
	glutWireCube(1);
	//glutSolidCube(1);
	glPopMatrix();
}
//=============================================================================================================