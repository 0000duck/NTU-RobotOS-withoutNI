#pragma once
#include "SerialClass.h"
#include <Eigen\Eigen\Dense>
#include <Windows.h>
#include <vector>

// 設定timer最小時間精度
#define TARGET_RESOLUTION 1    // 1-millisecond target resolution 

namespace sp{ // mean serial port
	class Motor{
	public:
		Motor();
		~Motor();
		void setInterpolation(int Pmin,int Pmax, double Rmin, double Rmax);
		void operator =(const double& Rref);
		const int& getPref() const;
	private:
		int _Pmin, _Pmax, _Pref; // P mean position
		double _Rmin, _Rmax; // R mean real angle
		int interpolation(const double& Rref);
	};

	class Driver{
	public:
		Driver();
		Driver(int i);
		~Driver();
		int size() const; // return size of _motor
		void addMotor();
		const Motor& operator[](int i) const;
		Motor& operator[](int i);
	private:
		std::vector<Motor*> _motor;
	};

	class Arduino{
		// default baud rate: 115200
	public:
		Arduino();
		~Arduino();
		Arduino(char* comPort);
		bool setComPort(char* comPort);
		bool isConnected() const;
		int driverSize() const; // return size of _driver
		int motorSize() const; // return size of motor
		const Driver& operator[](int i) const;
		Driver& operator[](int i);
		Motor& operator()(int i);
		void addDriver(int i = 0);
		void sendData(unsigned char* buffer, unsigned int nbByte);
	private:
		std::vector<Driver*> _driver;
		Serial *_serial;
	};
	//---------------------------------- adhoc ---------------------------------------------
	// global variable
	extern Arduino mega;
	extern std::vector<std::vector<int>> ePath;
	extern bool threadUsingFrag;
	void initMega();
	void pathR2E(const std::vector<std::vector<double>>& path, std::vector<std::vector<int>>& ePath);
	void pathR2E(const Eigen::MatrixXd& path, std::vector<std::vector<int>>& ePath);
	void addPathR2E(const std::vector<std::vector<double>>& path, std::vector<std::vector<int>>& ePath);
	void addPathR2E(const Eigen::MatrixXd& path, std::vector<std::vector<int>>& ePath);
	void megaSendPath();
	// for thread
	extern DWORD spThreadID;
	extern HANDLE spHandle;
	// thread callback function
	DWORD WINAPI spThread(LPVOID lpParameter);

	// timer callback function
	void CALLBACK spTimer(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dw1, DWORD dw2);
	extern int ePathIndex;
}