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

void error(const char * msg) {
	printf(msg);
	printf("\r\n");
	exit(1);
}
class Robot: public SampleRobot {

	Joystick m_stick;
	Victor SteerOut;
	Victor BrakeOut;
	Victor EnableControl;
	AnalogInput SteeringPot;
	AnalogOutput ThrottleOut;
	VardenEncoder leftWheel;
	VardenEncoder rightWheel;
	string FatalError = "";
	double lastCCCenter;

public:
	Robot():
		SteerOut(6),
		BrakeOut(5),
		EnableControl(4),
		SteeringPot(0),
		ThrottleOut(0),
		m_stick(0), // Initialize Joystick on port 0.
		leftWheel(3, 4, true, Encoder::EncodingType::k4X, 0.1, 0.02198), // 63 ticks/rev
		rightWheel(1, 2, true, Encoder::EncodingType::k1X, 0.1, 0.001333) // 1024 ticks/rev
	{
	}

	void Disabled() {
		printf("Disabled Mode\r\n");
	}

	void Autonomous() {
		printf("Auto mode\r\n");
		double _lastUpdate = 0;
		int sockfd;
		char buffer[256];
		struct sockaddr_in serv_addr;
		int n;
		double steer = 0, throttle = 0;
		printf("Bind new socket\r\n");
		sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (sockfd < 0)
			error("ERROR opening socket");
		bzero((char *) &serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(1180);
		if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
			error("ERROR on binding");
		while (IsAutonomous() && IsEnabled()) {
			EnableControl.Set(1);
			bzero(buffer, 256);
			n = recv(sockfd, buffer, 256, MSG_DONTWAIT);
			while(n > 0) {
				string str(buffer);
				int split = str.find(',');
				steer = atof(str.substr(0,split).c_str());
				throttle = atof(str.substr(split+1).c_str());

				_lastUpdate = Timer::GetFPGATimestamp();
				n = recv(sockfd, buffer, 256, MSG_DONTWAIT);
			}
			this->autonomousPeriodic(steer, throttle, _lastUpdate);

			Wait(0.01);
		}
		close(sockfd);
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
		SteerOut.Set((target-pos)*0.1);
	}

	void throttleTo(double target) {
		target = min(max(-1., target), 1.);
		double thr = 5*max(0., target);
		double brk = 0.55*max(0., -target);
		//cout << thr << " " << brk << endl;
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
		double proportional = 3*(targetSpeed - topSpeed)/TOP_SPEED;
		double output = feedForward + proportional;
		if (output < 0) {
			output -= 0.25;
		} else if (output < 0) {
			output = 0;
		}
		return output;
	}

	void autonomousPeriodic(double steer, double throttle, double lastUpdate){
		this->leftWheel.tick();
		this->rightWheel.tick();
		if (!this->FatalError.empty()) {
			cout << "FATAL ERROR: " << this->FatalError << endl;
			SteerOut.Set(0);
			throttleTo(-1);
		} else if (lastUpdate < Timer::GetFPGATimestamp() - 0.1) {
			cout << "Timed Out" << endl;
			SteerOut.Set(0);
			throttleTo(-1);
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
			EnableControl.Set(1);
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
