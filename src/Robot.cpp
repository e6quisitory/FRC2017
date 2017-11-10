#include <memory>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Robot.h"
#include "WPILib.h"
#include "AHRS.h"

int DrivingPosition;

Robot::Robot() {

}

void Robot::RobotInit() {
	CameraServer::GetInstance()->StartAutomaticCapture(0);
	CameraServer::GetInstance()->StartAutomaticCapture(1);
}

void Robot::DisabledInit() {
	DrivingPosition = 0;
	DrivingMode = 0;
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	//Resetting gyro angle when the robot is in disables mode
	ahrs->Reset();

	if (stick->GetRawButton(12)) {
		DrivingPosition = 1;

	}else if (stick->GetRawButton(10)) {
		DrivingPosition = 2;

	}else if (stick->GetRawButton(8)) {
		DrivingPosition = 3;

	}else if (stick->GetRawButton(9)){
		DrivingPosition = 0;

	}else if (stick->GetRawButton(11)) {
		DrivingPosition =4;
	}

	frc::SmartDashboard::PutNumber("AutoMode", DrivingPosition);
}

void Robot::AutonomousInit() {
	//intialzing encoders with right config. Function found in robot.h
	EncoderInit(this->Left_Encoder);
	EncoderInit(this->Right_Encoder);
}

void Robot::AutonomousPeriodic() {
	ShowData();
	//averaging the reading from both encoders and displaying
	//double encAvg = (Right_Encoder->GetDistance() + Left_Encoder->GetDistance())/2.0;
	//frc::SmartDashboard::PutNumber("Average Encoder Distance", encAvg);

	//double angle2 = (ahrs->GetYaw());
	//leftMotors((-(((0.01)*(angle2))+0.3)));
	//rightMotors((-(((-0.01)*(angle2))+0.3)));

	switch(DrivingPosition){
	case 1:
		LeftSideAutonomous();
		break;
	case 2:
		MiddleAutonomous();
		break;
	case 3:
		RightSideAutonomous();
		break;
	case 4:
		HopperTest();
		break;

	default:
		break;
	}
}

void Robot::TeleopInit() {

	if (autonomousCommand != nullptr) {
		autonomousCommand->Cancel();
	}
	Right_Encoder->Reset();
	Left_Encoder->Reset();
}

void Robot::TeleopPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	ShowData();
	double encAvg = ((Right_Encoder->GetDistance() + Left_Encoder->GetDistance())/2.0);
	frc::SmartDashboard::PutNumber("Average Encoder Distance", encAvg);
	frc::SmartDashboard::PutNumber("Left Encoder", Left_Encoder->GetDistance());
	frc::SmartDashboard::PutNumber("Right Distance", Right_Encoder->GetDistance());

	//drive system
	double throttle = stick->GetThrottle();
	double y = stick->GetRawAxis(1);
	double z = stick->GetRawAxis(2);

	//changes acceleration to slower
	z = z*((((-throttle)+1)/2));
	y = y*((((-throttle)+1)/2));

	//driving formula
	double leftmotor = 0.5*z - y;
	double rightmotor = 0.5*z + y;

	rightMotors(-rightmotor);
	leftMotors(leftmotor);

	//intake/outake system
	bool Outtake = stick->GetRawButton(1);
	bool Intake = stick->GetRawButton(2);
	bool Dump = stick->GetRawButton(3);

	double OuttakeForward = 1;
	double IntakeForward = 1;
	double IntakeReverse = -1;
	double DumpReverse = -1;

	if (Outtake) {
		intakemotor->Set(OuttakeForward);
		conveyormotor->Set(OuttakeForward);

	}else if (Intake) {
		intakemotor->Set(IntakeForward);
		conveyormotor->Set(IntakeReverse);
		outtakemotor->Set(IntakeForward);

	}else if (Dump){
		intakemotor->Set(DumpReverse);
		conveyormotor->Set(DumpReverse);
		outtakemotor->Set(DumpReverse);

	}else{
		intakemotor->Set(0);
		conveyormotor->Set(0);
		outtakemotor->Set(0);
	}

	//winch system
	bool WinchUp = stick->GetRawButton(5);

	if (WinchUp) {
		winchmotor->Set(-1);

	}else{
		winchmotor->Set(0);
	}
}

void Robot::TestInit() {

}

void Robot::TestPeriodic() {
	frc::LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
