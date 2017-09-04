#pragma once
#include "glut.h"

namespace gl{
	extern int WinNumber;   //�Ψө�m�����N�X 

	extern float trans_x;		// �۾�2D��m
	extern float trans_y;
	extern int old_tran_x;   //����U�ƹ��ɪ������y�� 
	extern int old_tran_y;

	extern int old_rot_x;   //����U�ƹ��ɪ������y�� 
	extern int old_rot_y;

	extern float distance;  //�b�����x�}(glTranslatef();)���ϥ� 
	extern int old_dis_y;   //����U�ƹ��ɪ������y�� 

	extern int rot_x;       //�즲�᪺�۹�y�СA�γo�M�w�n����X�� 
	extern int rot_y;

	extern int record_x;    //�����W�@�����઺���� 
	extern int record_y;

	extern int currentButton; //�����{�b���U���ƹ�

	void WindowSize(int, int);             //�t�d������ø�Ϥ��e����� 
	void Keyboard(unsigned char, int, int); //�����L��J 
	void SpecialKeyboard(int, int, int);    //�S����L��J
	void Mouse(int, int, int, int);         //����ƹ����U�M��}�ɪ��T�� 
	void MotionMouse(int, int);             //����ƹ����U�������T�� 
	void Display(void);                      //�yø
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

//glutSolidTorus(20, 30, 10, 10);	������
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
//gluQuadricDrawStyle(p, GLU_LINE); // �e��Wire
//gluCylinder(p, 10, 10, 4, 10, 10);
//gluDisk(p, 5, 10, 10, 10);
//void glRectd(x1,y1,x2,y2); �e���