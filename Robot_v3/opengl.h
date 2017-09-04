#pragma once
#include "glut.h"

namespace gl{
	extern int WinNumber;   //用來放置視窗代碼 

	extern float trans_x;		// 相機2D位置
	extern float trans_y;
	extern int old_tran_x;   //剛按下滑鼠時的視窗座標 
	extern int old_tran_y;

	extern int old_rot_x;   //剛按下滑鼠時的視窗座標 
	extern int old_rot_y;

	extern float distance;  //在平移矩陣(glTranslatef();)中使用 
	extern int old_dis_y;   //剛按下滑鼠時的視窗座標 

	extern int rot_x;       //拖曳後的相對座標，用這決定要旋轉幾度 
	extern int rot_y;

	extern int record_x;    //紀錄上一次旋轉的角度 
	extern int record_y;

	extern int currentButton; //紀錄現在按下的滑鼠

	void WindowSize(int, int);             //負責視窗及繪圖內容的比例 
	void Keyboard(unsigned char, int, int); //獲取鍵盤輸入 
	void SpecialKeyboard(int, int, int);    //特殊鍵盤輸入
	void Mouse(int, int, int, int);         //獲取滑鼠按下和放開時的訊息 
	void MotionMouse(int, int);             //獲取滑鼠按下期間的訊息 
	void Display(void);                      //描繪
	void timer(int t);
}

// basic plot function
// for glut
//glutSolidSphere(100, 100, 100);
//glutWireSphere(40, 10, 10);

//glutWireCube(40);
//glutSolidCube(40);

//glutSolidCone(10, 50, 10, 10);
//glutWireCone(10, 50, 10, 10);

//glutSolidTorus(20, 30, 10, 10);	甜甜圈
//glutWireTorus(20, 30, 100, 100);
//glutSolidDodecahedron();
//glutWireDodecahedron();

//glutSolidOctahedron();
//glutWireOctahedron();
//glutWireTetrahedron();
//glutWireIcosahedron();
//glutWireTeapot(10);

// for gl
//GLUquadricObj *p = gluNewQuadric();
//gluQuadricDrawStyle(p, GLU_LINE); // 畫成Wire
//gluCylinder(p, 10, 10, 4, 10, 10);
//gluDisk(p, 5, 10, 10, 10);
//void glRectd(x1,y1,x2,y2); 畫方形