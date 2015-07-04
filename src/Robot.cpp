#include "WPILib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <algorithm>

using namespace std;

void debug(const char *msg) {

}

void error(const char *msg) {
	printf(msg);
	printf("\r\n");
	exit(1);
}

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot {

	Joystick m_stick;
	Victor SteerOut;
	Victor BrakeOut;
	Victor EnableControl;
	AnalogInput SteeringPot;
	AnalogOutput ThrottleOut;

public:
	Robot():
		SteerOut(6),
		BrakeOut(5),
		EnableControl(4),
		SteeringPot(0),
		ThrottleOut(0),
		m_stick(0) // Initialize Joystick on port 0.
	{

	}

	void Disabled() {
		printf("Disabled Mode\r\n");
		/*while(true) {
			steerTo(1);
			Wait(0.01);
		}*/
	}

	void steerTo(double target) {
		target = min(max(-25.0, target), 25.0);
		double pos = SteeringPot.GetVoltage();
		if (pos < 0.5) {
			return;
		}
		pos = (pos-2.75)*-20;
		//printf("%f - %f\r\n", target, pos, (target-pos)*-0.1);
		SteerOut.Set((target-pos)*0.1);
	}

	void throttleTo(double target) {
		double thr = min(max(0., 5*target), 5.);
		double brk = min(max(0., -0.5*target), 0.4);
		cout << thr << " " << brk << endl;
		ThrottleOut.SetVoltage(thr);
		BrakeOut.Set(brk);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
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

				cout << str << " " << split
					<< " -- " << str.substr(0,split) << " " << str.substr(split+1)
					<< " -- " << steer << " " << throttle << endl;

				_lastUpdate = Timer::GetFPGATimestamp();
				n = recv(sockfd, buffer, 256, MSG_DONTWAIT);
			}

			if (Timer::GetFPGATimestamp() - _lastUpdate < 0.1) {
				steerTo(steer);
				throttleTo(throttle);
			} else {
				printf("Timed Out\r\n");
				SteerOut.Set(0);
				ThrottleOut.SetVoltage(0);
			}

			Wait(0.01);
		}
		close(sockfd);
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() {
		printf("Op Control");

		while (IsOperatorControl() && IsEnabled()) {
			EnableControl.Set(1);
			double x = m_stick.GetY();
			int sign = x>0?0:1;
			x = x*x*sign;
			throttleTo(x);
			Wait(0.01);
		}

	}

	/**
	 * Runs during test mode
	 */
	void Test() {
		printf("Test");
	}
};

START_ROBOT_CLASS(Robot);
