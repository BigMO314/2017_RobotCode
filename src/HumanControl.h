#pragma once

#include <GearManagement.h>
#include "MOLib.h"
#include "Robot Specifications.h"
#include "BallManagement.h"
#include "Climber.h"

namespace ControlPeriod{
	class HumanControl{
	private:
		//Defines the joysricks,drivetrains,shooter,Intakes,and gear thingeys.
		MOLib::XBoxController*	ctl_Driver;
		MOLib::XBoxController*	ctl_Operator;
		MOLib::TankDrivetrain*	rbt_Drivetrain;
		Robot::BallManagement*	rbt_BallManagement;
		Robot::GearManagement*	rbt_Gear;
		Robot::Climber*			rbt_Climber;
		MOLib::Vision::Target*	vis_Boiler;
		WPILib::Solenoid*		led_BoilerVision;

		WPILib::Timer			tmr_GearLight;
		WPILib::Timer			tmr_GearIntakeUp;

		bool m_GearLightState			= true;

		int m_CustomShooterSpeed = 0;

		MOLib::Dashboard::Number dsh_CustomShooterSpeed{"[Shooter] Custom Shooter Speed", 0.0};
		MOLib::Dashboard::Number dsh_ManualShooterSpeed{"[Shooter] Manual Shooter Speed", 0.0};

	public:
		HumanControl(
				MOLib::XBoxController* ref_Driver, MOLib::XBoxController* ref_Operator,
				MOLib::TankDrivetrain* ref_Drivetrain, Robot::BallManagement* ref_BallManagement,
				Robot::GearManagement* ref_Gear, Robot::Climber* ref_Climber, WPILib::Solenoid* ref_BoilerVision,
				MOLib::Vision::Target* ref_Boiler){
			this->ctl_Driver			= ref_Driver;
			this->ctl_Operator			= ref_Operator;

			this->rbt_Drivetrain		= ref_Drivetrain;
			this->rbt_BallManagement	= ref_BallManagement;
			this->rbt_Gear				= ref_Gear;
			this->rbt_Climber			= ref_Climber;
			this->led_BoilerVision		= ref_BoilerVision;
			this->vis_Boiler			= ref_Boiler;
			tmr_GearLight.Reset();
			tmr_GearLight.Start();
			tmr_GearIntakeUp.Reset();
			tmr_GearIntakeUp.Start();
		}

		void Update(){

			dsh_CustomShooterSpeed.Set(rbt_BallManagement->GetSpeedFromDistance());
			//SetCustomShooterSpeed(dsh_CustomShooterSpeed.Get());

			//Drivetrain Controls==============================================================================================================================
			rbt_Drivetrain->SetArcadeDrive(ctl_Driver->GetY(MOLib::XBoxController::kLeftHand), -ctl_Driver->GetX(MOLib::XBoxController::kRightHand));
			rbt_Drivetrain->SetShift((ctl_Driver->GetTriggerAxis(MOLib::XBoxController::kRightHand) ? MOLib::ShiftState::kHighSpeed : MOLib::ShiftState::kLowSpeed));

			//Climber==============================================================================================================================
			if(ctl_Driver->GetBumper(MOLib::XBoxController::kLeftHand)){ rbt_Climber->SetPower(0.15); }
			else if(ctl_Driver->GetTriggerAxis(MOLib::XBoxController::kLeftHand)){ rbt_Climber->SetPower(1.0); }
			else{ rbt_Climber->SetPower(0.0); }

			//Gear floor intake==============================================================================================================================
			if(ctl_Operator->GetAButton()){
				rbt_Gear->ExtendIntake();
				rbt_Gear->DisableIntake();
			}
			else if(ctl_Operator->GetBButton()) {
				if(rbt_Gear->GetGearLoaded()) {
					rbt_Gear->RetractIntake();
				}
				else {
					rbt_Gear->ExtendIntake();
				}
				rbt_Gear->EnableIntakeIn();
			}
			else if(ctl_Operator->GetXButton()){
				rbt_Gear->ExtendIntake();
				rbt_Gear->EnableIntakeOut();
			}
			else if(ctl_Operator->GetYButton()){
				rbt_Gear->RetractIntake();
				rbt_Gear->EnableIntakeIn();
				tmr_GearIntakeUp.Reset();
			}
			else if(ctl_Operator->GetTriggerAxis(MOLib::XBoxController::kRightHand) > 0.2){
				rbt_Gear->SetIntakePower(ctl_Operator->GetTriggerAxis(MOLib::XBoxController::kRightHand));
			}
			else{
				if(tmr_GearIntakeUp.Get() < 1){ rbt_Gear->EnableIntakeIn(); }
				else{ rbt_Gear->DisableIntake(); }
			}

			//Ball Indexing===============================================================================================================================
			if(ctl_Operator->GetAButton()){ rbt_BallManagement->EnableIndexer(); }
			else{ rbt_BallManagement->DisableIndexer(); }

			//Auto Align==================================================================================================================================
			if(ctl_Driver->GetAButton()){
				led_BoilerVision->Set(true);
				rbt_Drivetrain->AlignToGoal();
				rbt_Drivetrain->SetAutoAlignDrive(ctl_Driver->GetY(MOLib::XBoxController::kLeftHand));
				if(vis_Boiler->GetDistance() > 100 && vis_Boiler->GetDistance() < 130){
					m_CustomShooterSpeed = rbt_BallManagement->GetSpeedFromDistance();
				}
				else{
					m_CustomShooterSpeed = 4900;
				}
			}
			else{ led_BoilerVision->Set(false); }

			//Shooter==============================================================================================================================
			if(ctl_Operator->GetBumper(MOLib::XBoxController::kRightHand)){ rbt_BallManagement->SetShooterSpeed(4700); }
			else if(ctl_Operator->GetPOV() == 180){ rbt_BallManagement->SetShooterSpeed(0.0); }
			else if(ctl_Operator->GetPOV() == 270){ rbt_BallManagement->SetShooterSpeed(m_CustomShooterSpeed); }
			else if(ctl_Operator->GetStartButton() && ctl_Operator->GetBackButton()){ rbt_BallManagement->SetShooterSpeed(dsh_ManualShooterSpeed.Get()); }
		}
	};
}
