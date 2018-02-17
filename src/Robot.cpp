
#include "MOLib.h"
#include "Utility.h"
#include "Robot Specifications.h"

#include "Autonomous.h"
#include "BallManagement.h"
#include "Climber.h"
#include "GearManagement.h"
#include "HumanControl.h"

//TODO Create Simple Program to output all Encoder Values onto dashboard

class Onslaught: public frc::SampleRobot {
private:
	long double runCount = 0;

	//DriveTrain
	WPILib::Victor*					mtr_L_Drive_1;
	WPILib::Victor*					mtr_L_Drive_2;
	WPILib::Victor*					mtr_R_Drive_1;
	WPILib::Victor*					mtr_R_Drive_2;
	WPILib::Solenoid*				sol_Shifter;
	WPILib::Encoder*				enc_L_DriveDistance;
	WPILib::Encoder*				enc_R_DriveDistance;
	MOLib::AnalogGyro*				gyr_DriveAngle;

	MOLib::TankDrivetrain*			rbt_Drivetrain;

	//Ball Management
	WPILib::Victor*					mtr_L_Belting;
	WPILib::Victor*					mtr_R_Belting;
	WPILib::Solenoid*				sol_Agitator;
	WPILib::Victor*					mtr_Kicker;
	WPILib::Victor*					mtr_L_Uptake;
	WPILib::Victor*					mtr_R_Uptake;
	WPILib::Victor*					mtr_T_Shooter;
	WPILib::Victor*					mtr_B_Shooter_1;
	WPILib::Victor*					mtr_B_Shooter_2;
	WPILib::Encoder*				enc_T_ShooterSpeed;
	WPILib::Encoder*				enc_B_ShooterSpeed;
	WPILib::Solenoid*				sol_Hopper;

	MOLib::PIDLoop*					pid_T_ShooterSpeed;
	MOLib::PIDLoop*					pid_B_ShooterSpeed;

	Robot::BallManagement*			rbt_BallManagement;

	//Gear Management
	WPILib::Victor*					mtr_Intake;
	WPILib::Solenoid*				sol_Intake;
	WPILib::DigitalInput*			pho_IntakeLoaded;

	Robot::GearManagement* 			rbt_GearManagement;

	//Climber
	WPILib::Victor*					mtr_L_Climber;
	WPILib::Victor*					mtr_R_Climber;

	Robot::Climber*					rbt_Climber;

	//Vision
	MOLib::Vision::Target*			vis_Boiler;
	WPILib::Solenoid*				led_BoilerVision;

	//Controllers
	MOLib::XBoxController*			ctl_Driver;
	MOLib::XBoxController*			ctl_Operator;

	ControlPeriod::HumanControl*	prd_HumanControl;

	ControlPeriod::Autonomous*		prd_Autonomous;

	MOLib::Dashboard::Indicator*	dsh_DriveDistance_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveDistance_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveDistance_Distance;
	MOLib::Dashboard::Number*		dsh_DriveDistance_Plot;
	MOLib::Dashboard::Number*		dsh_DriveDistance_P;
	MOLib::Dashboard::Number*		dsh_DriveDistance_I;
	MOLib::Dashboard::Number*		dsh_DriveDistance_D;

	MOLib::Dashboard::Indicator*	dsh_DriveStraight_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveStraight_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveStraight_P;
	MOLib::Dashboard::Number*		dsh_DriveStraight_I;
	MOLib::Dashboard::Number*		dsh_DriveStraight_D;

	MOLib::Dashboard::Indicator*	dsh_DriveAngle_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveAngle_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveAngle_Angle;
	MOLib::Dashboard::Number*		dsh_DriveAngle_Plot;
	MOLib::Dashboard::Number*		dsh_DriveAngle_P;
	MOLib::Dashboard::Number*		dsh_DriveAngle_I;
	MOLib::Dashboard::Number*		dsh_DriveAngle_D;

	MOLib::Dashboard::Indicator*	dsh_DriveVision_PIDEnabled;
	MOLib::Dashboard::Indicator*	dsh_DriveVision_PIDOnTarget;
	MOLib::Dashboard::Number*		dsh_DriveVision_OffCenter;
	MOLib::Dashboard::Number*		dsh_DriveVision_Plot;
	MOLib::Dashboard::Number*		dsh_DriveVision_P;
	MOLib::Dashboard::Number*		dsh_DriveVision_I;
	MOLib::Dashboard::Number*		dsh_DriveVision_D;
	MOLib::Dashboard::Number*		dsh_DriveVision_Distance;

	MOLib::Dashboard::Number*		dsh_T_ShooterSpeed;
	MOLib::Dashboard::Number*		dsh_T_ShooterSpeed_P;
	MOLib::Dashboard::Number*		dsh_T_ShooterSpeed_I;
	MOLib::Dashboard::Number*		dsh_T_ShooterSpeed_D;

	MOLib::Dashboard::Number*		dsh_B_ShooterSpeed;
	MOLib::Dashboard::Number*		dsh_B_ShooterSpeed_P;
	MOLib::Dashboard::Number*		dsh_B_ShooterSpeed_I;
	MOLib::Dashboard::Number*		dsh_B_ShooterSpeed_D;

	//MOLib::Dashboard::Boolean*		dsh_ShooterInRange;

	MOLib::Dashboard::Indicator*	dsh_GearIntake_HasGear;


	WPILib::DigitalInput*			jmp_PracticeRobot;


public:
	Onslaught() {

		NetworkTable::GlobalDeleteAll();

		//Drivetrain
		this->mtr_L_Drive_1				= new WPILib::Victor(0);
		this->mtr_L_Drive_2				= new WPILib::Victor(1);
		this->mtr_R_Drive_1				= new WPILib::Victor(2);
		this->mtr_R_Drive_2				= new WPILib::Victor(3);
		this->sol_Shifter				= new WPILib::Solenoid(0);
		this->enc_L_DriveDistance		= new WPILib::Encoder(0,1);
		this->enc_R_DriveDistance		= new WPILib::Encoder(2,3);
		this->gyr_DriveAngle			= new MOLib::AnalogGyro(0);
		this->vis_Boiler				= new MOLib::Vision::Target("");

		rbt_Drivetrain					= new MOLib::TankDrivetrain(
			mtr_L_Drive_1, mtr_L_Drive_2, NullPtr,
			mtr_R_Drive_1, mtr_R_Drive_2, NullPtr,
			sol_Shifter, MOLib::ShiftState::kHighSpeed,
			enc_L_DriveDistance, gyr_DriveAngle, vis_Boiler
		);

		//Ball Management
		this->mtr_L_Belting				= new WPILib::Victor(12);
		this->mtr_R_Belting				= new WPILib::Victor(13);
		this->sol_Agitator				= new WPILib::Solenoid(5);
		this->mtr_Kicker				= new WPILib::Victor(6);
		this->mtr_L_Uptake				= new WPILib::Victor(7);
		this->mtr_R_Uptake				= new WPILib::Victor(8);
		this->mtr_T_Shooter				= new WPILib::Victor(9);
		this->mtr_B_Shooter_1			= new WPILib::Victor(10);
		this->mtr_B_Shooter_2			= new WPILib::Victor(11);
		this->sol_Hopper				= new WPILib::Solenoid(3);
		this->enc_T_ShooterSpeed		= new WPILib::Encoder(4,5);
		this->enc_B_ShooterSpeed		= new WPILib::Encoder(6,7);
		this->pid_T_ShooterSpeed		= new MOLib::PID::EncLoop(0.0, 0.0, 0.0, enc_T_ShooterSpeed);
		this->pid_B_ShooterSpeed		= new MOLib::PID::EncLoop(0.0, 0.0, 0.0, enc_B_ShooterSpeed);

		rbt_BallManagement				= new Robot::BallManagement(
			mtr_L_Belting, mtr_R_Belting, sol_Agitator,
			mtr_Kicker, mtr_L_Uptake, mtr_R_Uptake,
			mtr_T_Shooter, mtr_B_Shooter_1, mtr_B_Shooter_2,
			sol_Hopper,
			enc_T_ShooterSpeed, enc_B_ShooterSpeed,
			pid_T_ShooterSpeed, pid_B_ShooterSpeed, vis_Boiler
		);

		//Gear Management.
		this->mtr_Intake				= new WPILib::Victor(4);
		this->sol_Intake				= new WPILib::Solenoid(1);
		this->pho_IntakeLoaded			= new WPILib::DigitalInput(9);

		rbt_GearManagement				= new Robot::GearManagement(
			mtr_Intake, sol_Intake, pho_IntakeLoaded
		);

		led_BoilerVision				= new Solenoid(6);

		//Defining Climber.
		this->mtr_L_Climber				= new WPILib::Victor(14);
		this->mtr_R_Climber				= new WPILib::Victor(15);

		rbt_Climber						= new Robot::Climber(
			mtr_L_Climber, mtr_R_Climber
		);

		//Defining Controllers.
		this->ctl_Driver				= new MOLib::XBoxController(0, 0.1);
		this->ctl_Operator				= new MOLib::XBoxController(1, 0.1);


		prd_HumanControl				= new ControlPeriod::HumanControl(
			ctl_Driver, ctl_Operator,
			rbt_Drivetrain, rbt_BallManagement, rbt_GearManagement, rbt_Climber, led_BoilerVision, vis_Boiler
		);

		prd_Autonomous					= new ControlPeriod::Autonomous(rbt_Drivetrain, rbt_BallManagement, rbt_GearManagement, rbt_Climber,
				led_BoilerVision, vis_Boiler);

		dsh_DriveDistance_PIDEnabled	= new MOLib::Dashboard::Indicator("[DriveDistance] PID Enabled");
		dsh_DriveDistance_PIDOnTarget	= new MOLib::Dashboard::Indicator("[DriveDistance] PID on Target");
		dsh_DriveDistance_Distance		= new MOLib::Dashboard::Number("[DriveDistance] Distance");
		dsh_DriveDistance_Plot			= new MOLib::Dashboard::Number("[DriveDistance] Plot");
		dsh_DriveDistance_P				= new MOLib::Dashboard::Number("[DriveDistance] P", 0.023);
		dsh_DriveDistance_I				= new MOLib::Dashboard::Number("[DriveDistance] I", 1.0e-13);
		dsh_DriveDistance_D				= new MOLib::Dashboard::Number("[DriveDistance] D", 0.038);

		dsh_DriveStraight_PIDEnabled	= new MOLib::Dashboard::Indicator("[DriveStraight] PID Enabled");
		dsh_DriveStraight_PIDOnTarget	= new MOLib::Dashboard::Indicator("[DriveStraight] PID on Target");
		dsh_DriveStraight_P				= new MOLib::Dashboard::Number("[DriveStraight] P", 0.032);
		dsh_DriveStraight_I				= new MOLib::Dashboard::Number("[DriveStraight] I", 0.0);
		dsh_DriveStraight_D				= new MOLib::Dashboard::Number("[DriveStraight] D", 0.0);

		dsh_DriveAngle_PIDEnabled		= new MOLib::Dashboard::Indicator("[DriveAngle] PID Enabled");
		dsh_DriveAngle_PIDOnTarget		= new MOLib::Dashboard::Indicator("[DriveAngle] PID on Target");
		dsh_DriveAngle_Angle			= new MOLib::Dashboard::Number("[DriveAngle] Angle");
		dsh_DriveAngle_Plot				= new MOLib::Dashboard::Number("[DriveAngle] Plot");
		dsh_DriveAngle_P				= new MOLib::Dashboard::Number("[DriveAngle] P", 0.04);
		dsh_DriveAngle_I				= new MOLib::Dashboard::Number("[DriveAngle] I", 0.0);
		dsh_DriveAngle_D				= new MOLib::Dashboard::Number("[DriveAngle] D", 0.04);

		dsh_DriveVision_PIDEnabled		= new MOLib::Dashboard::Indicator("[DriveVision] PID Enabled");
		dsh_DriveVision_PIDOnTarget		= new MOLib::Dashboard::Indicator("[DriveVision] PID on Target");
		dsh_DriveVision_OffCenter		= new MOLib::Dashboard::Number("[DriveVision] Angle");
		dsh_DriveVision_Plot			= new MOLib::Dashboard::Number("[DriveVision] Plot");
		dsh_DriveVision_P				= new MOLib::Dashboard::Number("[DriveVision] P", 0.02);
		dsh_DriveVision_I				= new MOLib::Dashboard::Number("[DriveVision] I", 0.0);
		dsh_DriveVision_D				= new MOLib::Dashboard::Number("[DriveVision] D", 0.0);
		dsh_DriveVision_Distance		= new MOLib::Dashboard::Number("[DriveVision] Distance", 0.0);

		dsh_T_ShooterSpeed				= new MOLib::Dashboard::Number("[T ShooterSpeed] T ShooterSpeed");
		dsh_T_ShooterSpeed_P			= new MOLib::Dashboard::Number("[T ShooterSpeed] P", 0.000015);
		dsh_T_ShooterSpeed_I			= new MOLib::Dashboard::Number("[T ShooterSpeed] I", 0.002);
		dsh_T_ShooterSpeed_D			= new MOLib::Dashboard::Number("[T ShooterSpeed] D", 0.0);

		dsh_B_ShooterSpeed				= new MOLib::Dashboard::Number("[B ShooterSpeed] B ShooterSpeed");
		dsh_B_ShooterSpeed_P			= new MOLib::Dashboard::Number("[B ShooterSpeed] P", 0.000015);
		dsh_B_ShooterSpeed_I			= new MOLib::Dashboard::Number("[B ShooterSpeed] I", 0.002);
		dsh_B_ShooterSpeed_D			= new MOLib::Dashboard::Number("[B ShooterSpeed] D", 0.0);

		//dsh_ShooterInRange				= new MOLib::Dashboard::Boolean("[Shooter] Shooter In Range", false);

		dsh_GearIntake_HasGear			= new MOLib::Dashboard::Indicator("[Gear Intake] Has Gear", pho_IntakeLoaded->Get());

		jmp_PracticeRobot				= new WPILib::DigitalInput(8);


		SmartDashboard::PutData("Selected Alliance", &prd_Autonomous->chs_Alliance);
		SmartDashboard::PutData("Selected Auton", &prd_Autonomous->chs_Auton);

	}

	void UpdateDashboard(){
		dsh_DriveDistance_PIDEnabled->Set(rbt_Drivetrain->pid_DriveDistance->IsEnabled());
		dsh_DriveDistance_PIDOnTarget->Set(rbt_Drivetrain->pid_DriveDistance->OnTarget());
		dsh_DriveDistance_Distance->Set(enc_L_DriveDistance->GetDistance());
		dsh_DriveDistance_Plot->Set(enc_L_DriveDistance->GetDistance());

		dsh_DriveStraight_PIDEnabled->Set(rbt_Drivetrain->pid_DriveStraight->IsEnabled());
		dsh_DriveStraight_PIDOnTarget->Set(rbt_Drivetrain->pid_DriveStraight->OnTarget());

		dsh_DriveAngle_PIDEnabled->Set(rbt_Drivetrain->pid_DriveAngle->IsEnabled());
		dsh_DriveAngle_PIDOnTarget->Set(rbt_Drivetrain->pid_DriveAngle->OnTarget());
		dsh_DriveAngle_Angle->Set(gyr_DriveAngle->GetAngle());
		dsh_DriveAngle_Plot->Set(gyr_DriveAngle->GetAngle());

		dsh_DriveVision_PIDEnabled->Set(rbt_Drivetrain->IsVisionPIDEnabled());
		dsh_DriveVision_PIDOnTarget->Set(rbt_Drivetrain->IsVisionPIDAlignedToGoal());
		dsh_DriveVision_OffCenter->Set(vis_Boiler->GetAngle());
		dsh_DriveVision_Plot->Set(rbt_Drivetrain->GetVisionPIDError());
		dsh_DriveVision_Distance->Set(vis_Boiler->GetDistance());

		rbt_Drivetrain->pid_DriveDistance->SetPID(dsh_DriveDistance_P->Get(), dsh_DriveDistance_I->Get(), dsh_DriveDistance_D->Get());
		rbt_Drivetrain->pid_DriveStraight->SetPID(dsh_DriveStraight_P->Get(), dsh_DriveStraight_I->Get(), dsh_DriveStraight_D->Get());
		rbt_Drivetrain->pid_DriveAngle->SetPID(dsh_DriveAngle_P->Get(), dsh_DriveAngle_I->Get(), dsh_DriveAngle_D->Get());
		rbt_Drivetrain->pid_GoalAngle->SetPID(dsh_DriveVision_P->Get(), dsh_DriveVision_I->Get(), dsh_DriveVision_D->Get());
		rbt_BallManagement->SetBShooterPID(dsh_B_ShooterSpeed_P->Get(), dsh_B_ShooterSpeed_I->Get(), dsh_B_ShooterSpeed_D->Get());
		rbt_BallManagement->SetTShooterPID(dsh_T_ShooterSpeed_P->Get(), dsh_T_ShooterSpeed_I->Get(), dsh_T_ShooterSpeed_D->Get());

		dsh_B_ShooterSpeed->Set(rbt_BallManagement->GetBShooterSpeed());
		dsh_T_ShooterSpeed->Set(rbt_BallManagement->GetTShooterSpeed());

		//dsh_ShooterInRange->Set(dsh_DriveVision_Distance->Get() > 100 && dsh_DriveVision_Distance->Get() < 130);

		dsh_GearIntake_HasGear->Set(pho_IntakeLoaded->Get());
	}

	void RobotInit() {
		//Inverting the Right Side of the DriveTrain.
		mtr_L_Drive_1->SetInverted(false);
		mtr_L_Drive_2->SetInverted(false);
		mtr_R_Drive_1->SetInverted(true);
		mtr_R_Drive_2->SetInverted(true);

		rbt_Drivetrain->SetWheelDiameter(Robot::WheelDiameter);
		rbt_Drivetrain->SetGearRatio(Robot::LowGearRatio);
		rbt_Drivetrain->SetScale(0.91, 1.0, 1.0, 0.9);//0.91 1.0 1.0 0.95

		enc_R_DriveDistance->SetDistancePerPulse((Circumference(4.0) / 2048) * Robot::LowGearRatio);

		enc_L_DriveDistance->SetReverseDirection(true);

		enc_R_DriveDistance->SetReverseDirection(true);

		gyr_DriveAngle->Calibrate();
		gyr_DriveAngle->SetAngleScale(90.0 / 88.0);

		rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
		rbt_Drivetrain->pid_DriveDistance->SetPID(0.023, 1.0e-13, 0.038);
		rbt_Drivetrain->pid_DriveDistance->SetTargetTime(0.2);
		rbt_Drivetrain->pid_DriveDistance->SetAbsoluteTolerance(1.0);

		rbt_Drivetrain->pid_DriveStraight->SetOutputRange(-0.7, 0.7);
		rbt_Drivetrain->pid_DriveStraight->SetPID(0.032, 0.0, 0.0);
		rbt_Drivetrain->pid_DriveStraight->SetTargetTime(0.2);
		rbt_Drivetrain->pid_DriveStraight->SetAbsoluteTolerance(1.0);

		rbt_Drivetrain->pid_DriveAngle->SetOutputRange(-0.7, 0.7);
		rbt_Drivetrain->pid_DriveAngle->SetPID(0.04, 0.0, 0.04);
		rbt_Drivetrain->pid_DriveAngle->SetTargetTime(0.1);
		rbt_Drivetrain->pid_DriveAngle->SetAbsoluteTolerance(1.5);

		rbt_Drivetrain->pid_GoalAngle->SetOutputRange(-0.24, 0.24);
		rbt_Drivetrain->pid_GoalAngle->SetPID(0.02, 0.0, 0.0);//NOT SET!!                       .
		rbt_Drivetrain->pid_GoalAngle->SetTargetTime(0.1);
		rbt_Drivetrain->pid_GoalAngle->SetAbsoluteTolerance(2);

		mtr_Intake->SetInverted(jmp_PracticeRobot);
		mtr_Kicker->SetInverted(jmp_PracticeRobot->Get());
		mtr_L_Uptake->SetInverted(false);
		mtr_R_Uptake->SetInverted(true);

		mtr_T_Shooter->SetInverted(true);
		mtr_B_Shooter_1->SetInverted(true);
		mtr_B_Shooter_2->SetInverted(false);

		enc_T_ShooterSpeed->SetReverseDirection(true);
		enc_T_ShooterSpeed->SetDistancePerPulse(60.0/512);
		enc_T_ShooterSpeed->SetSamplesToAverage(127);
		//pid_T_ShooterSpeed->SetPID(0.00001, 0.002, 0.00001);
		pid_T_ShooterSpeed->SetPID(0.000015, 0.002, 0.0);//0.00002
		pid_T_ShooterSpeed->SetPIDSourceType(PIDSourceType::kRate);
		pid_T_ShooterSpeed->SetOutputRange(-1.0, 1.0);
		pid_T_ShooterSpeed->SetTargetTime(0.1);
		pid_T_ShooterSpeed->SetAbsoluteTolerance(0.1);

		enc_B_ShooterSpeed->SetReverseDirection(true);
		enc_B_ShooterSpeed->SetDistancePerPulse(60.0/512);
		enc_B_ShooterSpeed->SetSamplesToAverage(127);
		pid_B_ShooterSpeed->SetPID(0.000015, 0.002, 0.0);
		pid_B_ShooterSpeed->SetPIDSourceType(PIDSourceType::kRate);
		pid_B_ShooterSpeed->SetOutputRange(-1.0, 1.0);
		pid_B_ShooterSpeed->SetTargetTime(0.1);
		pid_B_ShooterSpeed->SetAbsoluteTolerance(0.1);

		vis_Boiler->SetResoloution(320, 240);
		vis_Boiler->SetTargetSize(15, 10);
		vis_Boiler->SetFOV(58.5, 45.6);

		CameraServer* server = CameraServer::GetInstance();
		cs::UsbCamera camera = server->StartAutomaticCapture();

		camera.SetResolution(160, 120);
		camera.SetFPS(20);

	}

	void Disabled() { Print("Robot Disabled"); }

	void Autonomous() {
		prd_Autonomous->AutonomousInit();
		vis_Boiler->SetCameraOffset(-10*(prd_Autonomous->chs_Alliance.GetSelected() == "Red Alliance" ? 1 : -1));//-10
		while (IsAutonomous() && IsEnabled()) {
			UpdateDashboard();
			prd_Autonomous->Update();
		}
	}

	void OperatorControl() override {
		enc_L_DriveDistance->Reset();
		enc_R_DriveDistance->Reset();
		gyr_DriveAngle->Reset();
		rbt_Drivetrain->StopAnglePID();
		rbt_Drivetrain->StopDistancePID();
		rbt_BallManagement->DisableShooterPID();
		rbt_BallManagement->SetShooterPower(0,0);
		rbt_BallManagement->RetractHopper();

		rbt_Drivetrain->SetShift(MOLib::ShiftState::kLowSpeed);

		vis_Boiler->SetCameraOffset(-10*(prd_Autonomous->chs_Alliance.GetSelected() == "Red Alliance" ? 1 : -1)-2);//-10

		while (IsOperatorControl() && IsEnabled()) {
			UpdateDashboard();
			prd_HumanControl->Update();
			rbt_Drivetrain->Update();
			rbt_BallManagement->Update();
			rbt_GearManagement->Update();
			rbt_Climber->Update();
			vis_Boiler->Update();
			frc::Wait(0.005);
		}
	}

	void Test() override {}
};
START_ROBOT_CLASS(Onslaught)
