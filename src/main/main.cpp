#include <iostream>
#include <fstream>
#include "opengl.h"
#include "glut.h"
#include "scenario.h"
using namespace std;
using namespace Eigen;

int main(int argc, char** argv){

	// scenario init
	loadBoundingBox("box.txt");
	graspPlanning();
	//releasePlanning("goto.txt");
	pathPrintOut();
	// opengl
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(500, 80);
	gl::WinNumber = glutCreateWindow("HTU Arm Hand Model");

	//下面五個是用來指定Callback函數 
	glutReshapeFunc(gl::WindowSize);
	glutKeyboardFunc(gl::Keyboard);
	glutSpecialFunc(gl::SpecialKeyboard);
	glutMouseFunc(gl::Mouse);
	glutMotionFunc(gl::MotionMouse);
	glutDisplayFunc(gl::Display);
	glutTimerFunc(10, gl::timer, 0);

	// GL while loop
	glutMainLoop();

	system("pause");
	return 0;
}