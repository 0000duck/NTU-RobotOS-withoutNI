#include "MyControlProject.h"
using namespace std;
#ifdef use_namespace
namespace ROBOOP {
	using namespace NEWMAT;
#endif

COpenGLControl pOpenGL;
Robot MyRobot;
MyControlProject::MyControlProject()
{
	try
	{
		Inital_Atom();
		COpenGLControl::glFlagATomStick = true;
		//pOpenGL.Draw_RobotArm(MyRobot);
	}
	catch (int err)
	{
		cout << "initial fail" << endl;
	}
}
MyControlProject::~MyControlProject()
{
}
void MyControlProject::Thead1Open()
{
	cout << "Thread 1 Open" << endl;
	while (1)
	{
	}
}
void MyControlProject::Inital_Atom()
{
	const Real Atom_DH[] =
	{
		// joint_type, theta, d,a,alpha, thetamin, thetamax, joint_offset,/**/ m, cm x, cm y, cm z, Ixx, Ixy, Ixz, Iyy,Iyz, Izz, lock
		0, 0, 1.22, 0, M_PI / 2, 0, 0, 0,/* */  1.74, 0, 0.01, 1.18,/* */ 0.00461, 0, 0, 0.00373, 0, 0.00226, 0,
		0, 0, 0, 0, M_PI / 2, 0, 0, M_PI,/* */ 0.353, -0.000563, 0.0189, 0.00805, /* */0.000314, 0, 0, 0.000276, 0, 0.000264, 0,
		0, 0, 0.371, 0.01, M_PI / 2.0, 0, 0, M_PI,/* */ 1.995, -0.004689, 0.00023, 0.2346,/* */ 0.0152, 0, 0, 0.0151, 0, 0.001, 0,
		0, 0, 0, -0.01, -M_PI / 2.0, 0, 0, -M_PI / 2,/* */ 0.236, 0.0347, 0.00691, 0.000406, /* */ 0.00023, 0, 0, 0.000305, 0, 0.000193, 0,
		0, 0, 0.28, 0, M_PI / 2.0, 0, 0, M_PI,/* */ 1.331, 0.00103, 0.000588, 0.1606,/* */ 0.00566, 0, 0, 0.00558, 0, 0.000569, 0,
		0, 0, 0, 0.1451, M_PI / 2.0, 0, 0, M_PI / 2.0,/* */ 0.274, 0.000177, 0.0371, -0.00124,/* */ 0.000198, 0, 0, 0.000124, 0, 0.000134, 0
	};
	const Real Atom_Motor[] =  // Im Gr B Cf [3863 3257 2642 2642 2642 2232]
	{
		1.2e-5, 200, 0, 0, // using + and - directions average
		1.2e-5, 250, 0, 0,
		1.1e-6, 200, 0, 0,
		1.1e-6, 250, 0, 0,
		1.1e-6, 200, 0, 0,
		3.8e-7, 300, 0, 0
	};
	Matrix initrobot, initrobotm;
	initrobot = Matrix(6, 19);
	initrobotm = Matrix(6, 4);
	initrobot << Atom_DH;
	initrobotm << Atom_Motor;
	MyRobot = Robot(initrobot, initrobotm);
	int dof = MyRobot.get_available_dof();
	//////////////////////////////////	
	cout << "Robot D-H parameters\n";
	cout << "theta     d       a     alpha\n";
	cout << setw(7) << setprecision(3) << initrobot.SubMatrix(1, dof, 2, 5);
	cout << "Atom DOF = " << gAtom_DOF << endl;
	cout << "\n";
	cout << "Robot D-H inertial parameters\n";
	cout << "  mass     cx       cy      cz     Ixx     Ixy     Ixz     Iyy     Iyz     Izz\n";
	cout << setw(7) << setprecision(3) << initrobot.SubMatrix(1, dof, 9, 18);
	cout << "\n";
	//cout << "Robot motors inertia, gear ratio, viscous and Coulomb friction coefficients\n";
	//cout << "  Im       Gr       B       Cf\n";
	//cout << setw(7) << setprecision(3) << initrobotm;
	cout << "\n";
	cout << "Robot joints variables\n";
	cout << setw(7) << setprecision(3) << MyRobot.get_q().as_row();
	cout << "\n";
	cout << "Robot Inertia matrix\n";
	cout << setw(7) << setprecision(3) << MyRobot.inertia(MyRobot.get_q());
	cout << "\n";
	cout << "Robot Gravity matrix\n";
	cout << setw(7) << setprecision(3) << MyRobot.G().as_row();
	cout << "\n";
	cout << "Robot Initial Torque\n";
	cout << setw(7) << setprecision(3) << MyRobot.torque(MyRobot.get_q(), MyRobot.get_q(), MyRobot.get_q()).as_row();
	cout << "\n";
	cout << "Atom Build Success!!" << endl;

	///  ¶}Thread 1
	mThread = new thread(&MyControlProject::Thead1Open,this);

}

void MyControlProject::MyTorqueControl()
{
	int dof = MyRobot.get_available_dof();
	ColumnVector qi(6), qf(6);
	Real qir[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; qi << qir;
	Real qfr[] = { 90.0/M_PI, 0.0, 0.0, 0.0, 0.0, 0.0 }; qf << qfr;



}