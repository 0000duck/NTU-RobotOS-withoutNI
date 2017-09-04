
#include "opengl.h"
#include <cmath>
#include "..\main\scenario.h"
#define GLUT_TOT_BUTTON 3
using namespace gl;

static double pi = acos(-1.0);

int gl::WinNumber = 0;
int gl::old_rot_x = 0;
int gl::old_rot_y = 0;
float gl::distance = 0;
int gl::old_dis_y = 0;
int gl::rot_x = 0;
int gl::rot_y = 0;
int gl::record_x = 0;
int gl::record_y = 0;
int gl::currentButton = GLUT_TOT_BUTTON;
float gl::trans_x = 0;
float gl::trans_y = 0;
int gl::old_tran_x = 0;
int gl::old_tran_y = 0;

void gl::Display(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);                            //用白色塗背景 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_BACK, GL_LINE);                            //設定面的背面用線條顯示 

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0, 950, 0, 0, 0, 0, 0, 0, 1);                    //視線的座標及方向 
	glTranslatef(-trans_x, distance, trans_y);
	glRotatef((float)rot_y + (float)record_y, -1.0, 0.0, 0.0);   //以x軸當旋轉軸 
	glRotatef((float)rot_x + (float)record_x, 0.0, 0.0, 1.0);   //以z軸當旋轉軸 
	scenarioShow();
	glutSwapBuffers();
}

void gl::Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case char(13) :
		motionFlag = true;
		ntuRobot.reset();
		break;
	default:
		break;
	}
	glutPostRedisplay();            //令視窗重繪
}

void gl::WindowSize(int w, int h)
{
	float rate;
	if (h == 0) h = 1;                        //阻止h為零，分母可不能為零啊 
	glViewport(0, 0, w, h);                 //當視窗長寬改變時，畫面也跟著變 
	rate = (float)w / (float)h;                //畫面視野變了，但內容不變形 

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, rate, 1.0, 5000.0);   //透視投影 
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void gl::Mouse(int button, int state, int x, int y)
{
	switch (button){
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_UP)
		{
			record_x += x - old_rot_x;
			record_y += y - old_rot_y;

			rot_x = 0;   //沒有歸零會有不理想的結果 
			rot_y = 0;

			currentButton = GLUT_TOT_BUTTON;
		}
		else // GLUT_DOWN
		{
			old_rot_x = x;
			old_rot_y = y;
			currentButton = GLUT_LEFT_BUTTON;
		}
		break;
	case GLUT_MIDDLE_BUTTON:
		if (state == GLUT_UP)
		{
			currentButton = GLUT_TOT_BUTTON;
		}
		else // GLUT_DOWN
		{
			old_dis_y = y;
			currentButton = GLUT_MIDDLE_BUTTON;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_UP)
		{
			currentButton = GLUT_TOT_BUTTON;
		}
		else // GLUT_DOWN
		{
			currentButton = GLUT_RIGHT_BUTTON;
			old_tran_x = x;
			old_tran_y = y;
		}
		break;
	}
}

void gl::MotionMouse(int x, int y)
{
	switch (currentButton){
	case GLUT_LEFT_BUTTON:
		rot_x = x - old_rot_x;
		rot_y = y - old_rot_y;
		break;
	case GLUT_MIDDLE_BUTTON:
		distance -= (y - old_dis_y);
		old_dis_y = y;
		break;
	case GLUT_RIGHT_BUTTON:
		trans_x += (x - old_tran_x);
		trans_y -= (y - old_tran_y);
		old_tran_x = x;
		old_tran_y = y;
		break;
	}
	glutPostRedisplay();
}

void gl::SpecialKeyboard(int key, int x, int y){
	glutPostRedisplay();
}

void gl::timer(int t){
	glutTimerFunc(10, timer, 0);
	static int segment = 0;
	static int iter = 0;
	if (motionFlag){
		if (segment < path.size()){
			if (iter < path[segment].rows()){
				for (int i = 0, n = path[segment].cols(); i < n; ++i){
					ntuRobot[i].setCmd(path[segment](iter, i));
				}
				iter++;
			}
			else{
				segment++;
				iter = 0;
			}
		}
		else{
			segment = 0;
			iter = 0;
			motionFlag = false;
		}
	}
	glutPostRedisplay();
}