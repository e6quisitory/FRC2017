#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include <Encoder.h>
#include <SampleRobot.h>
#include <IterativeRobot.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Timer.h>
#include <Joystick.h>
#include "iostream"
#include "RobotMap.h"

#include "WPILib.h"
#include "AHRS.h"

class Robot: public frc::IterativeRobot {

private:

	int DrivingMode = 0;

	//declaring gyro named "ahrs"
	AHRS *ahrs = new AHRS(SPI::Port::kMXP);
	double angle = ahrs->GetYaw();

	//drive system
	VictorSP *right = new VictorSP(0);
	VictorSP *left = new VictorSP (5);
	VictorSP *right2 = new VictorSP (1);
	VictorSP *left2 = new VictorSP (6);
	VictorSP *right3 = new VictorSP(2);
	VictorSP *left3 = new VictorSP (7);

	VictorSP *conveyormotor = new VictorSP (3);
	VictorSP *intakemotor = new VictorSP (4);
	VictorSP *outtakemotor = new VictorSP (8);
	VictorSP *winchmotor = new VictorSP (9);

	Joystick *stick = new Joystick(0);

	Encoder *Right_Encoder = new Encoder(1, 0, false, Encoder::EncodingType::k4X);
	Encoder *Left_Encoder = new Encoder(2, 3, false, Encoder::EncodingType::k4X);

	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestInit() override;
	void TestPeriodic() override;

	void ShowData(){
		double leftEncoder = Left_Encoder->GetDistance();
		double rightEncoder = Right_Encoder->GetDistance();
		double avgEnc = ((leftEncoder + rightEncoder) /2.0);
		float angle = ahrs->GetYaw();
		double speed = right->Get();

		frc::SmartDashboard::PutNumber("Average Encoder Distance", avgEnc);
		frc::SmartDashboard::PutNumber("Right Encoder Distance", rightEncoder);
		frc::SmartDashboard::PutNumber("Left Encoder Distance", leftEncoder);
		frc::SmartDashboard::PutNumber("Angle", angle);
		frc::SmartDashboard::PutNumber("Speed", speed);
	}
	//function taking in one value and applying to all speed controllers
	void allMotors(double x){
		rightMotors(x);
		leftMotors(x);
	}
	//function taking in one value and applying to only right motors. Negative value being applied because right motors are inverted
	void rightMotors(double x){
		right->Set(-x);
		right2->Set(-x);
		right3->Set(-x);
	}
	//function taking in one value and applying to only left motors
	void leftMotors(double x){
		left->Set(x);
		left2->Set(x);
		left3->Set(x);
	}
	//function taking in one encoder and configuring it with proper settings
	void EncoderInit(Encoder* enc){
		enc->SetMaxPeriod(.1);
		enc->SetMinRate(10);
		enc->SetDistancePerPulse(0.001440);
		enc->SetReverseDirection(true);
		enc->SetSamplesToAverage(7);
		enc->Reset();
	}

	void ResetAll(){
		ahrs->Reset();
		Left_Encoder->Reset();
		Right_Encoder->Reset();
	}

	double AbsoluteValue(double value){
		return (value < 0 ? value * -1.0 : value);
	}

	bool Straight(double targetDistance, double direction, double speed){
		double leftEncoder = Left_Encoder->GetDistance();
		double rightEncoder = Right_Encoder->GetDistance();
		double avgEnc = 1.14625*((leftEncoder + rightEncoder) /2.0);

		double angle = ahrs->GetYaw();

		if (avgEnc < AbsoluteValue(targetDistance)){
			leftMotors(direction*((((0.01)*(angle))+speed)));
			rightMotors(direction*((((-0.01)*(angle))+speed)));
			return false;
		}else{
			allMotors(0);
			return true;
		}
	}

	bool Turn(double targetDegrees, double maxspeed, double offset){
		float angle1 = ahrs->GetYaw();
		double absoluteangle = AbsoluteValue(angle1);
		double absolutetargetDegrees = AbsoluteValue(targetDegrees);
		double error = AbsoluteValue(angle1 - targetDegrees);
		frc::SmartDashboard::PutNumber("Error", error);
		//double Speed = ((-1.0/(3.0*absolutetargetDegrees))*absoluteangle)+(1.0/3.0);
		double Speed = (maxspeed)*(1-(absoluteangle/absolutetargetDegrees));

		if (angle<targetDegrees && error>0.9){
			rightMotors(-Speed-offset);
			leftMotors(Speed+offset);
			return false;

		}else if (angle>targetDegrees && error>0.9){
			rightMotors(Speed+offset);
			leftMotors(-Speed-offset);
			return false;
		}else{
			allMotors(0);
			return true;
		}
	}

	void LeftSideAutonomous() {
		switch (DrivingMode) {
		case 0:
			if (Straight(2.0, -1.0, 0.3) == true) {
				DrivingMode = 1;
			}
			break;

		case 1:
			if (Turn(45, 0.4, 0.13) == true) {
				Wait(1);
				DrivingMode = 2;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
				ahrs->Reset();
			}
			break;

		case 2:
			if (Straight(0.645, -1.0, 0.3) == true) {
				DrivingMode = 3;
			}
			break;

		case 3:
			if (Turn(0, 0.25, 0.13) == true) {
				DrivingMode = 4;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
			}
			break;

		case 4:
			if (Straight(0.35, -1.0, 0.3) == true) {
				DrivingMode = 5;
			}
			break;

		default:
			break;
		}
	}

	void RightSideAutonomous() {
		switch (DrivingMode) {
		case 0:
			if (Straight(1.92, -1.0, 0.3) == true) {
				DrivingMode = 1;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
			}
			break;

		case 1:
			if (Turn(-45, 0.4, 0.13) == true) {
				Wait(1);
				DrivingMode = 2;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
				ahrs->Reset();
			}
			break;

		case 2:
			if (Straight(0.6, -1.0, 0.3) == true) {
				DrivingMode = 3;
			}
			break;

		case 3:
			if (Turn(0, 0.4, 0.13) == true) {
				DrivingMode = 4;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
			}
			break;

		case 4:
			if (Straight(0.35, -1.0, 0.3) == true) {
				DrivingMode = 5;
			}
			break;

		default:
			break;
		}
	}

	void MiddleAutonomous() {
		switch (DrivingMode) {
		case 0:
			if (Straight(2.0, -1.0, 0.35) == true) {
				DrivingMode = 1;
			}
			break;

		default:
			break;
		}
	}

	void HopperTest() {
		switch (DrivingMode) {
		case 0:
			if (Straight(2.0, -1.0, 0.35) == true) {
				DrivingMode = 1;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
			}
			break;

		case 1:
			if (Turn(90, 0.3, 0.195) == true) {
				DrivingMode = 2;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
				ahrs->Reset();
			}
			break;

		case 2:
			if (Straight(0.5, -1.0, 0.35) == true) {
				DrivingMode = 3;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
				ahrs->Reset();
			}
			break;

		case 3:
			if (Turn(90, 0.3, 0.195) == true) {
				DrivingMode = 2;
				Left_Encoder->Reset();
				Right_Encoder->Reset();
				ahrs->Reset();
			}
			break;


		default:
			break;

		}
	}

public:
	Robot();

	void RobotInit();

	void OperatorControl();
};

#endif /* SRC_ROBOT_H_ */
