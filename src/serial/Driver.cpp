
#include "Driver.h"
#include <iostream>
#include <cmath>
using namespace sp;

static const double pi = acos(-1.0);

sp::Motor::Motor(){

}

sp::Motor::~Motor(){

}

void sp::Motor::setInterpolation(int Pmin, int Pmax, double Rmin, double Rmax){
	_Pmin = Pmin;
	_Pmax = Pmax;
	_Rmin = Rmin;
	_Rmax = Rmax;
}
void sp::Motor::operator =(const double& Rref){
	if (Rref > _Rmax){
		_Pref = interpolation(_Rmax);
	}
	else if (Rref < _Rmin){
		_Pref = interpolation(_Rmin);
	}
	else{
		_Pref = interpolation(Rref);
	}
}
int sp::Motor::interpolation(const double& Rref){
	return (int)(((Rref-_Rmin)*(_Pmax-_Pmin)/(_Rmax-_Rmin))+_Pmin);
}

const int& sp::Motor::getPref() const{
	return _Pref;
}

sp::Driver::Driver(){

}

sp::Driver::Driver(int i){
	_motor.reserve(i);
	for (int j = 0; j < i; ++j){
		_motor.push_back(new Motor());
	}
}

sp::Driver::~Driver(){
	for (int i = 0, n = _motor.size(); i < n; ++i){
		delete _motor[i];
	}
}
int sp::Driver::size() const{
	return _motor.size();
}
void sp::Driver::addMotor(){
	_motor.push_back(new Motor());
}

const Motor& sp::Driver::operator[](int i) const{
	return *_motor[i];
}

Motor& sp::Driver::operator[](int i){
	return *_motor[i];
}

sp::Arduino::Arduino(){
	_serial = 0;
}
sp::Arduino::~Arduino(){
	for (int i = 0, n = _driver.size(); i < n; ++i){
		delete _driver[i];
	}
	delete _serial;
}
sp::Arduino::Arduino(char* comPort){
	_serial = new Serial(comPort);
}

bool sp::Arduino::setComPort(char* comPort){
	_serial = new Serial(comPort);
	return _serial->IsConnected();
}

bool sp::Arduino::isConnected() const{
	return _serial->IsConnected();
}

int sp::Arduino::driverSize() const{
	return _driver.size();
}

int sp::Arduino::motorSize() const{
	int total = 0;
	for (int i = 0, n = _driver.size(); i < n; ++i){
		total += _driver[i]->size();
	}
	return total;
}

Motor& sp::Arduino::operator()(int i){
	int j = 0;
	for (int k = 0, n = _driver.size(); k < n; ++k){
		j = _driver[k]->size();
		if (j > i){
			return (*this)[k][i];
		}
		else{
			i = i - j;
		}
	}
	return (*this)[0][0];
}

const Driver& sp::Arduino::operator[](int i) const{
	return *_driver[i];
}

Driver& sp::Arduino::operator[](int i){
	return *_driver[i];
}

void sp::Arduino::addDriver(int i){
	_driver.push_back(new Driver(i));
}

void sp::Arduino::sendData(unsigned char* buffer, unsigned int nbByte){
	_serial->WriteData(buffer, nbByte);
}

DWORD sp::spThreadID;
HANDLE sp::spHandle;
int sp::ePathIndex = 0;

DWORD WINAPI sp::spThread(LPVOID lpParameter){
	//TIMECAPS tc;

	//// ���o�w��timer�]�w�귽
	//if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) != TIMERR_NOERROR)
	//{
	//	// Error; application can't continue.  
	//	return 0;
	//}

	//unsigned int wTimerRes = min(max(tc.wPeriodMin, TARGET_RESOLUTION), tc.wPeriodMax);

	//timeBeginPeriod(wTimerRes);

	//unsigned int Period_time = 5; // Ĳ�o�g�� (millisecond)

	//unsigned int timeSetEventRslt = timeSetEvent(
	//	Period_time, //Ĳ�o���g���ɶ�
	//	TARGET_RESOLUTION, //timer�̤p�ɶ����
	//	sp::spTimer, //Callback function �W��
	//	wTimerRes, //�s��Τᴣ�Ѫ��^�ռƾ�
	//	TIME_PERIODIC | TIME_CALLBACK_FUNCTION | TIME_KILL_SYNCHRONOUS // �]�wtimer�������Ѽ�flag
	//	);

	//if (timeSetEventRslt == NULL)
	//	MessageBox(NULL, "TimeSetEvent fail. Maybe delay is not in the range of the minimum and maximum ", "���~", MB_OK);
	//int indexMax = ePath.size();
	//while (ePathIndex < indexMax){

	//}
	//timeEndPeriod(wTimerRes);         //����timeBeginPeriod  
	//timeKillEvent(timeSetEventRslt);  //����timeSetEventRslt  
	// index & flag �k�s
	ePathIndex = 0;
	threadUsingFrag = 0;
	return 0;
}

void CALLBACK sp::spTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2){
	static unsigned char buffer[14] = { 0 };
	for (int i = 0, n = ePath[ePathIndex].size(); i < n; ++i){
		buffer[2 * i] = ePath[ePathIndex][i] >> 8;
		buffer[2 * i + 1] = ePath[ePathIndex][i];
	}
	mega.sendData(buffer, 14);
	ePathIndex++;
}

Arduino sp::mega;

void sp::initMega(){
	mega.addDriver(1);
	mega[0][0].setInterpolation(0, 2522, 0, pi / 2);
	mega.addDriver(2);
	mega[1][0].setInterpolation(991, 680, 0, pi / 2);
	mega[1][1].setInterpolation(0, 2126, 0, pi * 99 / 180);
	mega.addDriver(2);
	mega[2][0].setInterpolation(962, 558, 0, pi / 2);
	mega[2][1].setInterpolation(0, 2328, 0, pi * 99 / 180);
	mega.addDriver(2);
	mega[3][0].setInterpolation(978, 622, 0, pi / 2);
	mega[3][1].setInterpolation(0, 2170, 0, pi * 99 / 180);
}

void sp::pathR2E(const std::vector<std::vector<double>>& path, std::vector<std::vector<int>>& ePath){
	ePath.clear();
	for (int i = 0, n = path.size(); i < n; ++i){
		ePath.push_back(std::vector<int>());
		ePath[i].reserve(mega.motorSize());
		for (int j = 0, m = path[i].size(); j < m; ++j){
			mega(j) = path[i][j];
			ePath[i].push_back(mega(j).getPref());
		}
	}
}

void sp::pathR2E(const Eigen::MatrixXd& path, std::vector<std::vector<int>>& ePath){
	ePath.clear();
	for (int i = 0, n = path.rows(); i < n; ++i){
		ePath.push_back(std::vector<int>());
		ePath[i].reserve(mega.motorSize());
		for (int j = 0, m = path.cols(); j < m; ++j){
			mega(j) = path(i, j);
			ePath[i].push_back(mega(j).getPref());
		}
	}
}

void sp::addPathR2E(const std::vector<std::vector<double>>& path, std::vector<std::vector<int>>& ePath){
	for (int i = 0, n = path.size(); i < n; ++i){
		ePath.push_back(std::vector<int>());
		ePath.back().reserve(mega.motorSize());
		for (int j = 0, m = path[i].size(); j < m; ++j){
			mega(j) = path[i][j];
			ePath.back().push_back(mega(j).getPref());
		}
	}
}
void sp::addPathR2E(const Eigen::MatrixXd& path, std::vector<std::vector<int>>& ePath){
	for (int i = 0, n = path.rows(); i < n; ++i){
		ePath.push_back(std::vector<int>());
		ePath.back().reserve(mega.motorSize());
		for (int j = 0, m = path.cols(); j < m; ++j){
			mega(j) = path(i, j);
			ePath.back().push_back(mega(j).getPref());
		}
	}
}

std::vector<std::vector<int>> sp::ePath;

void sp::megaSendPath(){
	if (threadUsingFrag == 0){
		spHandle = CreateThread(0, 0, spThread, 0, 0, &spThreadID);
		threadUsingFrag = 1;
	}
	else{
		std::cout << "Thread now using..., send the ePath next time!" << std::endl;
	}
}

bool sp::threadUsingFrag = 0;