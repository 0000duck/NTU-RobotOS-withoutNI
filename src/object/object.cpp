
#include "object.h"
#include <iostream>
using namespace obj;

void obj::BoundingBox::draw() const{
	const double &x1 = _min(0), &y1 = _min(1), &z1 = _min(2);
	const double &x2 = _max(0), &y2 = _max(1), &z2 = _max(2);
	const double delx = x2 - x1, dely = y2 - y1, delz = z2 - z1;
	glBegin(GL_LINE_STRIP);
	glVertex3d(x1/1000, y1/1000, z1/1000+1.22);
	glVertex3d(x1/1000, y2/1000, z1/1000+1.22);
	glVertex3d(x1/1000, y2/1000, z2/1000+1.22);
	glVertex3d(x1/1000, y1/1000, z2/1000+1.22);
	glVertex3d(x1/1000, y1/1000, z1/1000+1.22);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex3d(x1/1000, y1/1000, z2/1000+1.22);
	glVertex3d(x2/1000, y1/1000, z2/1000+1.22);
	glVertex3d(x2/1000, y2/1000, z2/1000+1.22);
	glVertex3d(x1/1000, y2/1000, z2/1000+1.22);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex3d(x1/1000, y1/1000, z1/1000+1.22);
	glVertex3d(x2/1000, y1/1000, z1/1000+1.22);
	glVertex3d(x2/1000, y2/1000, z1/1000+1.22);
	glVertex3d(x1/1000, y2/1000, z1/1000+1.22);
	glEnd();
	glBegin(GL_LINES);
	glVertex3d(x2/1000, y1/1000, z1/1000+1.22);
	glVertex3d(x2/1000, y1/1000, z2/1000+1.22);
	glVertex3d(x2/1000, y2/1000, z1/1000+1.22);
	glVertex3d(x2/1000, y2/1000, z2/1000+1.22);
	glEnd();
	//glPushMatrix();
	//glTranslated(_center(0)/1000, _center(1)/1000, _center(2)/1000);
	//	Draw_Cute_Axis(15);
	//glPopMatrix();
}

void obj::Draw_Cute_Axis(float LINK_LENGTH){
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