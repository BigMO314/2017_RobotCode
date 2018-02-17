#pragma once
#include <GearManagement.h>
#include "MOLib.h"
#include "BallManagement.h"
#include "Climber.h"

#define StageMachine	switch(AutonStage)
#define stage			case
#define NextStage		AutonStage++; break

namespace ControlPeriod{

	class Autonomous{
	private:
		MOLib::TankDrivetrain* rbt_Drivetrain;
		Robot::BallManagement* rbt_BallManagement;
		Robot::GearManagement* rbt_Gear;
		Robot::Climber* rbt_Climber;
		MOLib::Vision::Target* vis_Boiler;

		WPILib::Solenoid* vis_BoilerVision;

		WPILib::Timer tmr_Autonomous;
		WPILib::Timer tmr_GearMech;

		int runCount = 0;
		int AutonStage;
		int m_SelectedAlliance = 1;
		double m_LivoniaShooterPower = 0.0;
		std::string m_SelectedAuton;



	public:
		WPILib::SendableChooser<std::string>	chs_Alliance;

		WPILib::SendableChooser<std::string>	chs_Auton;
		const std::string kDoNothing								= "Do Nothing";
		const std::string kDriveForward								= "Drive Forward";
		const std::string kAutoAlignHopper60Ball					= "Auto Align Hopper 60 Ball";
		const std::string kAutoAlignTriggerHopper60Ball				= "Auto Align Trigger Hopper 60 Ball";
		const std::string kBoilerGear_AutoAlign10Ball				= "Boiler Gear : Auto Align 10 Ball";
		const std::string kCenterGear_AutoAlign10Ball				= "Center Gear : Auto Align 10 Ball";
		const std::string kCenterGear_AutoAlign10BallCrossField		= "Center Gear : Auto Align 10 Ball : Cross Field";
		const std::string kRetrievalGear_CrossField					= "Retrieval Gear : Cross Field";
		const std::string kCenterGear_AnotherOne					= "Center Gear : Another One";

		Autonomous(MOLib::TankDrivetrain* ref_Drivetrain, Robot::BallManagement* ref_BallManagement,
				Robot::GearManagement* ref_GearManagement, Robot::Climber* ref_Climber,
				WPILib::Solenoid* ref_BoilerVision, MOLib::Vision::Target* ref_Boiler){
			this->rbt_Drivetrain = ref_Drivetrain;
			this->rbt_BallManagement = ref_BallManagement;
			this->rbt_Gear = ref_GearManagement;
			this->rbt_Climber = ref_Climber;
			this->AutonStage = 0;
			this->vis_BoilerVision = ref_BoilerVision;
			this->vis_Boiler = ref_Boiler;


			chs_Alliance.AddDefault("Red Alliance", "Red Alliance");
			chs_Alliance.AddObject("Blue Alliance", "Blue Alliance");

			//SmartDashboard::PutData("Selected Alliance", &chs_Alliance);

			chs_Auton.AddDefault(kDoNothing, kDoNothing);
			chs_Auton.AddObject(kDriveForward, kDriveForward);
			chs_Auton.AddObject(kAutoAlignHopper60Ball, kAutoAlignHopper60Ball);
			chs_Auton.AddObject(kAutoAlignTriggerHopper60Ball, kAutoAlignTriggerHopper60Ball);
			chs_Auton.AddObject(kBoilerGear_AutoAlign10Ball, kBoilerGear_AutoAlign10Ball);
			chs_Auton.AddObject(kCenterGear_AutoAlign10Ball, kCenterGear_AutoAlign10Ball);
			chs_Auton.AddObject(kCenterGear_AutoAlign10BallCrossField, kCenterGear_AutoAlign10BallCrossField);
			chs_Auton.AddObject(kRetrievalGear_CrossField, kRetrievalGear_CrossField);
			chs_Auton.AddObject(kCenterGear_AnotherOne, kCenterGear_AnotherOne);

			//SmartDashboard::PutData("Selected Auton", &chs_Auton);
		}
		void AutonomousInit(){
			AutonStage = 0;
			m_SelectedAuton		= chs_Auton.GetSelected();
			m_SelectedAlliance	= (chs_Alliance.GetSelected() == "Red Alliance" ? 1 : -1);

			m_LivoniaShooterPower = (chs_Alliance.GetSelected() == "Red Alliance" ? 0.70 : 0.71);

			tmr_Autonomous.Reset();
			tmr_Autonomous.Start();

			tmr_GearMech.Reset();
			tmr_GearMech.Start();

			rbt_Drivetrain->StopAnglePID();
			rbt_Drivetrain->StopDistancePID();
			rbt_Drivetrain->ResetDistance();
			rbt_Drivetrain->ResetAngle();

			rbt_BallManagement->DisableShooterPID();
			rbt_BallManagement->SetShooterPower(0, 0);

			rbt_BallManagement->RetractHopper();
			rbt_BallManagement->DisableIndexer();

			vis_BoilerVision->Set(false);

			rbt_Drivetrain->SetShift(MOLib::ShiftState::kLowSpeed);
		}

		void Update(){
					 if(m_SelectedAuton == kDoNothing)									Auton_DoNothing();
				else if(m_SelectedAuton == kDriveForward)								Auton_DriveForward();
				else if(m_SelectedAuton == kAutoAlignHopper60Ball)						Auton_AutoAlignHopper60Ball();
				else if(m_SelectedAuton == kAutoAlignTriggerHopper60Ball)				Auton_AutoAlignTriggerHopper60Ball();
				else if(m_SelectedAuton == kBoilerGear_AutoAlign10Ball)					Auton_BoilerGear_AutoAlign10Ball();
				else if(m_SelectedAuton == kCenterGear_AutoAlign10Ball)					Auton_CenterGear_AutoAlign10Ball();
				else if(m_SelectedAuton == kCenterGear_AutoAlign10BallCrossField)		Auton_CenterGear_AutoAlign10BallCrossField();
				else if(m_SelectedAuton == kCenterGear_AnotherOne)						Auton_CenterGear_AnotherOne();
				else if(m_SelectedAuton == kRetrievalGear_CrossField)					Auton_RetrievalGear_CrossField();
				else{ Print("Error: Unknown or no auton selected"); }
			rbt_Drivetrain->Update();
			rbt_BallManagement->Update();
			rbt_Gear->Update();
			rbt_Climber->Update();
		}

	private:
		void Auton_DoNothing(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Do Nothing]");	NextStage;

				stage  1: Print("--Extending Hopper Sides");			NextStage;
				stage  2: rbt_Climber->SetPower(0.5);
						  rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->DisableShooter();
						  tmr_Autonomous.Reset();						NextStage;
				stage  3: if(tmr_Autonomous.Get() > 2)					NextStage;
				stage  4: rbt_Climber->Disable();						NextStage;
				stage  5: Print("Autonomous Complete [Do Nothing]");	NextStage;
			}
		};

		void Auton_DriveForward(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Drive Forward]");								NextStage;

				stage  1: Print("--Driving Forwards 87");											NextStage;
				stage  2: rbt_Climber->SetPower(0.5);
						  rbt_Drivetrain->GoToDistance(7.0_ft + 3.0_in);
						  tmr_Autonomous.Reset();													NextStage;
				stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.5)			NextStage;

				stage  4: Print("--Stopping Drivetrain and Systems");								NextStage;
				stage  5: rbt_Climber->Disable();
						  rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->SetShooterSpeed(0.0);
						  rbt_BallManagement->DisableIndexer();										NextStage;
				stage  6: Print("Autonomous Complete [Drive Forward]");								NextStage;
			}
		}

		void Auton_AutoAlignHopper60Ball(){
			StageMachine{
				stage  0: Print("Starting Autnomous [AutoAlighHopperShot]");										NextStage;

				stage  1: Print("--Ramping Up Shooter Wheels");
						  Print("--Driving Backwards 138 inches");
						  Print("--Extending Sides");																NextStage;
				stage  2: rbt_BallManagement->SetShooterSpeed(4900);
						  rbt_Drivetrain->GoToDistance(-138);
						  rbt_Climber->SetPower(0.5);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.2)							NextStage;

				stage  4: Print("--Turning 24 Degrees");															NextStage;
				stage  5: rbt_Drivetrain->GoToAngle(-24*m_SelectedAlliance);//24	35
						  rbt_Climber->SetPower(0.0);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.0)								NextStage;//1.0	1.1

				stage  7: Print("--Extending Intake");
						  Print("--Driving forward 71 inches");														NextStage;
				stage  8: rbt_Gear->ExtendIntake();
						  rbt_Drivetrain->GoToDistance(71);//71		59
						  tmr_Autonomous.Reset();																	NextStage;
				stage  9: if(tmr_Autonomous.Get() > 1.0)															NextStage;
				stage 10: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.5)							NextStage;

				stage 11: Print("--Turning on Vision Ring LEDs");													NextStage;
				stage 12: vis_BoilerVision->Set(true);
						  tmr_Autonomous.Reset();																	NextStage;

				stage 13: Print("--AutoAligning to Goal");
						  Print("--AutoAdjusting Wheel Speeds");
						  Print("--Enabling Indexer");																NextStage;
				stage 14: rbt_BallManagement->EnableIndexer();
						  rbt_Drivetrain->AlignToGoal();
						  rbt_Drivetrain->SetAutoAlignDrive(0);
						  if(vis_Boiler->GetDistance() > 100 && vis_Boiler->GetDistance() < 130){
							  rbt_BallManagement->SetShooterSpeed(rbt_BallManagement->GetSpeedFromDistance() - 100);
						  }
						  else{
							  rbt_BallManagement->SetShooterSpeed(4900-100);
						  }
						  if(tmr_Autonomous.Get() > 1.25)															NextStage;

				stage 15: Print("--Stopping Drivetrain");															NextStage;
				stage 16: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  tmr_Autonomous.Reset();																	NextStage;

				stage 17: Print("--Waiting on Auton Timeout");														NextStage;
				stage 18: rbt_BallManagement->EnableIndexer(); if(tmr_Autonomous.Get() > 14)						NextStage;

				stage 19: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->SetShooterSpeed(0.0);
						  rbt_BallManagement->DisableIndexer();														NextStage;
			}
		}

		void Auton_AutoAlignTriggerHopper60Ball(){
			StageMachine{
				stage  0: Print("Starting Autonomous [AutoAlignTriggerHopper60Bal]");								NextStage;

				stage  1: Print("--Ramping Up Shooter Wheels");
						  Print("--Extending Hopper");
						  Print("--Driving to 96 Inches");															NextStage;
				stage  2: //rbt_BallManagement->SetShooterSpeed(4900);
						  rbt_Climber->SetPower(0.5);
						  rbt_Drivetrain->GoToDistance(-100);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.5)							NextStage;

				stage  4: Print("--Triggering Hopper");																NextStage;
				stage  5: rbt_Climber->SetPower(0.0);
						  rbt_BallManagement->ExtendHopper();
						  tmr_Autonomous.Reset();																	NextStage;
				stage  6: if(tmr_Autonomous.Get() > 0.5)															NextStage;

				stage  7: Print("--Turning 25 Degrees");															NextStage;
				stage  8: rbt_Drivetrain->GoToAngle(-25*m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  9: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1)								NextStage;

				stage 10: rbt_Drivetrain->SetTankDrive(0.0, 0.0);													NextStage;
			}
		}

		void Auton_BoilerGear_AutoAlign10Ball(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Retrieval Side Gear]");										NextStage;

				stage  1: Print("--Driving backwards to 8 feet and 3 inches");										NextStage;
				stage  2: rbt_BallManagement->SetShooterSpeed(4900);
						  rbt_Climber->SetPower(0.5);
						  rbt_Gear->RetractIntake();
						  rbt_Drivetrain->GoToDistance(-7.0_ft + -2.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)							NextStage;

				stage  4: Print("--Turning to 57 Degrees");															NextStage;
				stage  5: rbt_Climber->Disable();
						  rbt_Drivetrain->GoToAngle(-57.0 * m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.25)							NextStage;

				stage  7: Print("--Driving Backwards 3 feet into Gear Hook");										NextStage;
				stage  8: rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.5, 0.5);								NextStage;
				stage  9: rbt_Drivetrain->GoToDistance(-3.0_ft + -3.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 10: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.25)							NextStage;

				stage 11: Print("--Release gear and drive forward 2 feet");											NextStage;
				stage 12: rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 13: if(tmr_Autonomous.Get() > 1.0)															NextStage;

				stage 14: rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(4.0_ft + 6.0_in);
						  tmr_Autonomous.Reset();			 														NextStage;
				stage 15: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)							NextStage;

				stage 16: Print("--Turning to 60 Degrees");															NextStage;
				stage 17: rbt_Drivetrain->GoToAngle(25.0 * m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 18: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.5)								NextStage;

				stage 19: vis_BoilerVision->Set(true);
						  tmr_Autonomous.Reset();																	NextStage;

				stage 20: rbt_Drivetrain->SetAutoAlignDrive(0);
							  rbt_BallManagement->SetShooterSpeed(rbt_BallManagement->GetSpeedFromDistance());

						  if(tmr_Autonomous.Get() > 0.5)															NextStage;
				stage 21: rbt_Gear->ExtendIntake();
						  rbt_BallManagement->EnableIndexer();
						  rbt_Drivetrain->SetAutoAlignDrive(0);
							  rbt_BallManagement->SetShooterSpeed(rbt_BallManagement->GetSpeedFromDistance());
						  if(tmr_Autonomous.Get() > 1.5)															NextStage;

				stage 22: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 23: rbt_BallManagement->EnableIndexer();
						  if(tmr_Autonomous.Get() > 15)																NextStage;
				stage 24: Print("Autonomous Completed [GearSide]");
						  Print("--Closing gear holder");
						  Print("--Stopping all systems");															NextStage;
				stage 25: rbt_Drivetrain->SetTankDrive(0.0, 0.0); rbt_BallManagement->SetShooterSpeed(0.0);			NextStage;
			}
		}

		void Auton_CenterGear_AutoAlign10Ball(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Center Gear]");												NextStage;

				stage  1: rbt_BallManagement->SetShooterSpeed(4700);
						  rbt_Climber->SetPower(0.5);																NextStage;
				stage  2: Print("--Driving backwards to 7ft 2in");
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.65, 0.65);							NextStage;
				stage  3: rbt_Drivetrain->GoToDistance(-7.0_ft + -2.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  4: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.75)							NextStage;

				stage  5: rbt_Climber->SetPower(0.0);
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  tmr_Autonomous.Reset();																	NextStage;

				stage  6: if(tmr_Autonomous.Get() > 0.5)															NextStage;

				stage  7: rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(3.0_ft + 8.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  8: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.5)							NextStage;

				stage  9: rbt_Drivetrain->GoToAngle(-73*m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 10: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.0)								NextStage;

				stage 11: rbt_Gear->ExtendIntake();
						  rbt_Drivetrain->GoToDistance(5.0_ft + 10.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 12: if(tmr_Autonomous.Get() > 1.25)															NextStage;

				stage 13: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 14: rbt_BallManagement->EnableIndexer(); if(tmr_Autonomous.Get() > 15)																NextStage;

				stage 15: Print("Autonomous Completed [GearSide]");
						  Print("--Closing gear holder");
						  Print("--Stopping all systems");															NextStage;
				stage 16: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->SetShooterSpeed(0.0);													NextStage;
			}
		}

		void Auton_CenterGear_AutoAlign10BallCrossField(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Center Gear]");												NextStage;

				stage  1: rbt_BallManagement->SetShooterSpeed(4700);
						  rbt_Climber->SetPower(0.5);																NextStage;
				stage  2: Print("--Driving backwards to 7ft 2in");
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.65, 0.65);							NextStage;
				stage  3: rbt_Drivetrain->GoToDistance(-7.0_ft + -2.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  4: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.75)							NextStage;

				stage  5: rbt_Climber->SetPower(0.0);
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  tmr_Autonomous.Reset();																	NextStage;

				stage  6: if(tmr_Autonomous.Get() > 0.5)															NextStage;

				stage  7: rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(3.0_ft + 8.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  8: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.25)							NextStage;

				stage  9: rbt_Drivetrain->GoToAngle(-73*m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 10: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 0.85)							NextStage;

				stage 11: rbt_Gear->ExtendIntake();
						  rbt_Drivetrain->GoToDistance(5.0_ft + 8.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 12: if(tmr_Autonomous.Get() > 1.15)															NextStage;

				stage 13: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 14: rbt_BallManagement->EnableIndexer(); if(tmr_Autonomous.Get() > 4)							NextStage;

				stage 15: rbt_BallManagement->DisableShooterPID();
						  rbt_BallManagement->SetShooterPower(0.0, 0.0);
						  rbt_Drivetrain->GoToAngle(75*m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 16: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1)								NextStage;

				stage 17: rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  rbt_Drivetrain->GoToDistance(-20.0_ft + -0.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 18: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 10)							NextStage;

				stage 19: Print("Autonomous Completed [GearSide]");
						  Print("--Closing gear holder");
						  Print("--Stopping all systems");															NextStage;
				stage 20: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_BallManagement->SetShooterSpeed(0.0);													NextStage;
			}
		}

		void Auton_RetrievalGear_CrossField(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Retrieval Side Gear]");										NextStage;

				stage  1: Print("--Driving backwards to 8 feet and 3 inches");										NextStage;
				stage  2: rbt_Climber->SetPower(0.5);
						  rbt_Gear->RetractIntake();
						  rbt_Drivetrain->GoToDistance(-7.0_ft + -6.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  3: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)							NextStage;

				stage  4: Print("--Turning to 57 Degrees");															NextStage;
				stage  5: rbt_Climber->Disable();
						  rbt_Drivetrain->GoToAngle(57.0 * m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  6: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.25)							NextStage;

				stage  7: Print("--Driving Backwards 3 feet into Gear Hook");										NextStage;
				stage  8: rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.5, 0.5);								NextStage;
				stage  9: rbt_Drivetrain->GoToDistance(-3.0_ft);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 10: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.25)							NextStage;

				stage 11: Print("--Release gear and drive forward 2 feet");											NextStage;
				stage 12: rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 13: if(tmr_Autonomous.Get() > 1.0)															NextStage;

				stage 14: rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(4.0_ft);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 15: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)							NextStage;

				stage 16: Print("--Turning to 60 Degrees");																							NextStage;
				stage 17: rbt_Drivetrain->GoToAngle(-60.0 * m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 18: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.5)								NextStage;

				stage 19: rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(-24.0_ft);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 20: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 5.0)							NextStage;

				stage 21: tmr_Autonomous.Reset();																	NextStage;
				stage 22: if(tmr_Autonomous.Get() > 15)																NextStage;
				stage 23: Print("Autonomous Completed [GearSide]");
						  Print("--Closing gear holder");
						  Print("--Stopping all systems");															NextStage;
				stage 24: rbt_Drivetrain->SetTankDrive(0.0, 0.0); rbt_BallManagement->SetShooterSpeed(0.0);			NextStage;
			}
		}

		void Auton_CenterGear_AnotherOne(){
			StageMachine{
				stage  0: Print("Starting Autonomous [Center Gear]");												NextStage;

				stage  1: rbt_Gear->DisableIntake();
						  rbt_Gear->RetractIntake();
						  rbt_Climber->SetPower(0.5);																NextStage;
				stage  2: Print("--Driving backwards to 7ft 2in");
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.65, 0.65);							NextStage;
				stage  3: rbt_Drivetrain->GoToDistance(-7.0_ft + -2.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  4: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.75)							NextStage;

				stage  5: rbt_Climber->SetPower(0.0);
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  tmr_Autonomous.Reset();																	NextStage;

				stage  6: if(tmr_Autonomous.Get() > 0.5)															NextStage;

				stage  7: rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(3.0_ft + 8.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage  8: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.5)							NextStage;

				stage  9: rbt_Drivetrain->GoToAngle(-90*m_SelectedAlliance);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 10: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.0)								NextStage;

				stage 11: rbt_Gear->ExtendIntake();
						  rbt_Gear->EnableIntakeIn();
						  rbt_Drivetrain->GoToDistance(5.0_ft + 8.0_in);
						  tmr_Autonomous.Reset();																	NextStage;
				stage 12: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.3)							NextStage;

				stage 13: tmr_GearMech.Reset();
						  tmr_Autonomous.Reset();
						  rbt_Gear->RetractIntake();
						  rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(-5.0_ft + -8.0_in);											NextStage;

				stage 14: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.5){ AutonStage++; }
						  if(tmr_GearMech.Get() > 1){ rbt_Gear->DisableIntake(); }									break;

				stage 15: tmr_Autonomous.Reset();
						  rbt_Drivetrain->GoToAngle(-90*m_SelectedAlliance);										NextStage;

				stage 16: if(rbt_Drivetrain->IsAtAngle() || tmr_Autonomous.Get() > 1.3){ AutonStage++; }
						  if(tmr_GearMech.Get() > 1){ rbt_Gear->DisableIntake(); }									break;

				stage 17: rbt_Gear->DisableIntake();
						  rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.65, 0.65);							NextStage;

				stage 18: rbt_Drivetrain->GoToDistance(3.0_ft + 8.0_in);
						  tmr_Autonomous.Reset();																	NextStage;

				stage 19: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 1.75)							NextStage;

				stage 20: rbt_Drivetrain->pid_DriveDistance->SetOutputRange(-0.9, 0.9);
						  rbt_Gear->ExtendIntake();
						  rbt_Gear->EnableIntakeOut();
						  tmr_Autonomous.Reset();																	NextStage;

				stage 21: if(tmr_Autonomous.Get() > 0.5)															NextStage;
				stage 22: tmr_Autonomous.Reset();
						  rbt_Drivetrain->StopDistancePID();
						  rbt_Drivetrain->GoToDistance(-4.0_ft);													NextStage;

				stage 23: if(rbt_Drivetrain->IsAtDistance() || tmr_Autonomous.Get() > 2.0)							NextStage;
				stage 24: rbt_Drivetrain->SetTankDrive(0.0, 0.0);
						  rbt_Gear->DisableIntake();																NextStage;
			}
			Print(AutonStage);
		}

	};
}
