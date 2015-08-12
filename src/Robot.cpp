#include "WPILib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <algorithm>
#include <iomanip>
#include "VardenEncoder.cpp"

using namespace std;

#define TOP_SPEED 6.5
#define TIMEOUT 0.08

void error(const char * msg) {
	printf(msg);
	printf("\r\n");
	exit(1);
}
class Robot: public SampleRobot {

	Joystick m_stick;
	CANTalon BrakeOut;
	CANTalon SteerOut;
	Relay EnableControl;
	AnalogInput SteeringPot;
	AnalogOutput ThrottleOut;
	VardenEncoder leftWheel;
	VardenEncoder rightWheel;
	string FatalError = "";
	double lastCCCenter;

public:
	Robot():
		BrakeOut(0),
		SteerOut(1),
		EnableControl(0),
		SteeringPot(0),
		ThrottleOut(0),
		m_stick(0), // Initialize Joystick on port 0.
		leftWheel(3, 4, true, Encoder::EncodingType::k4X, 0.1, 0.02198), // 63 ticks/rev
		rightWheel(1, 2, true, Encoder::EncodingType::k1X, 0.1, 0.001333) // 1024 ticks/rev
	{
	}

	void Disabled() {
		EnableControl.Set(Relay::kOff);
		printf("Disabled Mode\r\n");
	}

	void Autonomous() {
		printf("Auto mode\r\n");
		int data_sock, enable_sock;
		char data_buffer[256], enable_buffer[256];
		struct sockaddr_in data_addr, enable_addr;

		enable_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (enable_sock < 0)
			error("ERROR opening data socket");
		bzero((char *) &enable_addr, sizeof(enable_addr));
		enable_addr.sin_family = AF_INET;
		enable_addr.sin_addr.s_addr = INADDR_ANY;
		enable_addr.sin_port = htons(1170);
		if (bind(enable_sock, (struct sockaddr *) &enable_addr, sizeof(enable_addr)) < 0)
			error("ERROR on binding data socket");


		data_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (data_sock < 0)
			error("ERROR opening data socket");
		bzero((char *) &data_addr, sizeof(data_addr));
		data_addr.sin_family = AF_INET;
		data_addr.sin_addr.s_addr = INADDR_ANY;
		data_addr.sin_port = htons(1180);
		if (bind(data_sock, (struct sockaddr *) &data_addr, sizeof(data_addr)) < 0)
			error("ERROR on binding data socket");

		int n;
		double last_update=0, last_enable=0;
		double steer = 0, throttle = 0;

		while (IsAutonomous() && IsEnabled()) {
			EnableControl.Set(Relay::kForward);

			bzero(enable_buffer, 256);
			n = recv(enable_sock, enable_buffer, 256, MSG_DONTWAIT);
			while(n > 0) {
				if (enable_buffer[0] == '1') {
					last_enable = Timer::GetFPGATimestamp();
				} else {
					last_enable = 0;
				}

				n = recv(enable_sock, enable_buffer, 256, MSG_DONTWAIT);
			}

			bzero(data_buffer, 256);
			n = recv(data_sock, data_buffer, 256, MSG_DONTWAIT);
			while(n > 0) {
				string str(data_buffer);
				int split = str.find(',');
				steer = atof(str.substr(0,split).c_str());
				throttle = atof(str.substr(split+1).c_str());

				last_update = Timer::GetFPGATimestamp();
				n = recv(data_sock, data_buffer, 256, MSG_DONTWAIT);
			}
			this->autonomousPeriodic(steer, throttle, last_enable, last_update);

			Wait(0.01);
		}

		close(enable_sock);
		close(data_sock);
	}


	void steerTo(double target) {
		target = min(max(-25.0, target), 25.0);
		double pos = SteeringPot.GetVoltage();
		if (pos < 0.5) {
			this->FatalError = "Pot Disconnected";
			return;
		}
		pos = (pos-2.75)*-20;
		//printf("%f - %f\r\n", target, pos, (target-pos)*-0.1);
		SteerOut.Set((target-pos)*0.25);
	}

	void throttleTo(double target) {
		double thr, brk;
		target = min(max(-1., target), 1.);
		if (target > 0) {
			thr = 5*target;
			brk = -0.05;
		} else {
			thr = 0;
			brk = 0.55*-target;
		}
		cout << thr << " " << brk << endl;
		ThrottleOut.SetVoltage(thr);
		BrakeOut.Set(brk);
	}

	double cruiseControl(double targetSpeed) {
		double lt_left = this->leftWheel.GetLongTermRate();
		double lt_right = this->rightWheel.GetLongTermRate();
		if (abs(lt_left-lt_right) > (0.25*max(lt_left, lt_right) + 0.2*TOP_SPEED)) {
			cout << "WARNING: Encoders returning wildly different values";
		}

		double left = this->leftWheel.GetRate();
		double right = this->rightWheel.GetRate();
		double topSpeed = max(left, right);
		double feedForward = targetSpeed/TOP_SPEED;
		double proportional = 1*(targetSpeed - topSpeed)/TOP_SPEED;
		double output = feedForward + proportional;
		if (output < 0) {
			output -= 0.25;
		}
		return output;
	}

	void autonomousPeriodic(double steer, double throttle, double last_enable, double last_update){
		this->leftWheel.tick();
		this->rightWheel.tick();
		if (!this->FatalError.empty()) {
			cout << "FATAL ERROR: " << this->FatalError << endl;
			SteerOut.Set(0);
			throttleTo(-1);
		} else if (min(last_update, last_enable) < Timer::GetFPGATimestamp() - 0.1) {
			SteerOut.Set(0);
			throttleTo(-1);
			if (last_update < Timer::GetFPGATimestamp() - 0.1) {
				cout << "Timed Out" << endl;
			}
			if (last_enable < Timer::GetFPGATimestamp() - 0.1) {
				cout << "E-stopped" << endl;
			}
		} else {
			steerTo(steer);
			if (throttle > 0) {
				throttle = this->cruiseControl(throttle);
			}
			throttleTo(throttle);
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() {
		printf("Op Control");
		double setPoint = 0;
		cout << std::fixed << std::setprecision(2);

		while (IsOperatorControl() && IsEnabled()) {
			this->leftWheel.tick();
			this->rightWheel.tick();
			EnableControl.Set(Relay::kForward);
			double live = -m_stick.GetY();
			if(m_stick.GetRawButton(1)) {
				setPoint = live;
			}
			double output = this->cruiseControl(setPoint*TOP_SPEED);

			cout << live << " " << setPoint << " " << output << " " << (this->leftWheel.GetRate()+this->rightWheel.GetRate())/(2*TOP_SPEED) << endl;
			throttleTo(output);
			Wait(0.01);
		}

	}

	/**
	 * Runs during test mode
	 */
	void Test() {
		printf("Test");

		while (IsTest() && IsEnabled()) {
			cout << this->leftWheel.GetRate() << " " << this->rightWheel.GetRate() << endl;
			Wait(0.02);
			this->leftWheel.tick();
			this->rightWheel.tick();
		}
	}
};

START_ROBOT_CLASS(Robot);
