#pragma once
#include "MOLib.h"

namespace Robot{
	class BallManagement{
	private:
		//Shooter
		WPILib::Victor*				mtr_L_Belting;
		WPILib::Victor*				mtr_R_Belting;
		WPILib::Solenoid*	sol_Agitator;
		WPILib::Victor*		mtr_Kicker;
		WPILib::Victor*		mtr_L_Uptake;
		WPILib::Victor*		mtr_R_Uptake;

		WPILib::Victor*		mtr_T_Shooter;
		WPILib::Victor*		mtr_B_Shooter_1;
		WPILib::Victor*		mtr_B_Shooter_2;

		WPILib::Solenoid*	sol_Hopper;

		WPILib::Encoder*	enc_T_ShooterSpeed;
		WPILib::Encoder*	enc_B_ShooterSpeed;
		MOLib::PIDLoop*		pid_T_ShooterSpeed;
		MOLib::PIDLoop*		pid_B_ShooterSpeed;

		MOLib::Vision::Target*	vis_Boiler;

		WPILib::Timer		tmr_BeaterBar;

		float				m_L_Belting			= 0.0;
		float				m_R_Belting			= 0.0;

		bool				m_BeaterBar			= false;

		float				m_KickerPower		= 0.0;
		float				m_UptakePower		= 0.0;

		bool				m_Hopper			= false;

		float				m_T_ShooterPower	= 0.0;
		float				m_B_ShooterPower	= 0.0;

	public:
		/**
		 *
		 * @param ref_T_Shooter-making a reference for the Top Shooter.
		 * @param ref_B_Shooter_1-making a reference for one of the bottom shooter motors.
		 * @param ref_B_Shooter_2-making a reference for one of the bottom shooter motors.
		 * @param ref_R_Indexer-making a reference for the right indexeing motor.
		 * @param ref_L_Indexer-making a reference for the left indexeing motor.
		 * @param ref_C_Indexer-making a reference for the center indexeing motor.
		 */
		BallManagement(
			Victor* ref_L_Belting, Victor* ref_R_Belting, Solenoid* ref_Agitator,
				Victor* ref_Kicker, Victor* ref_L_Uptake, Victor* ref_R_Uptake,
				Victor* ref_T_Shooter, Victor* ref_B_Shooter_1 , Victor* ref_B_Shooter_2,
				Solenoid* ref_Hopper,
				Encoder* ref_T_ShooterSensor, Encoder* ref_B_ShooterSensor,
				MOLib::PIDLoop* ref_T_ShooterSpeed, MOLib::PIDLoop* ref_B_ShooterSpeed, MOLib::Vision::Target* ref_Boiler
				){
			this->mtr_L_Belting			= ref_L_Belting;
			this->mtr_R_Belting			= ref_R_Belting;
			this->sol_Agitator			= ref_Agitator;
			this->mtr_Kicker			= ref_Kicker;
			this->mtr_L_Uptake			= ref_L_Uptake;
			this->mtr_R_Uptake			= ref_R_Uptake;

			this->mtr_T_Shooter			= ref_T_Shooter;
			this->mtr_B_Shooter_1		= ref_B_Shooter_1;
			this->mtr_B_Shooter_2		= ref_B_Shooter_2;

			this->sol_Hopper			= ref_Hopper;

			this->enc_T_ShooterSpeed	= ref_T_ShooterSensor;
			this->enc_B_ShooterSpeed	= ref_B_ShooterSensor;

			this->pid_T_ShooterSpeed	= ref_T_ShooterSpeed;
			this->pid_B_ShooterSpeed	= ref_B_ShooterSpeed;

			this->vis_Boiler			= ref_Boiler;

			tmr_BeaterBar.Start();
		}

		void SetBeltingPower(float power){ m_L_Belting = -power; m_R_Belting = power; }

		void ExtendBeaterBar() { m_BeaterBar = true; }
		void RetractBeaterBar() { m_BeaterBar = false; }
		void ToggleBeaterBar() { m_BeaterBar = !m_BeaterBar; }
		bool GetBeaterBarState() { return m_BeaterBar; }

		void SetKickerPower(float power) { m_KickerPower = power; }
		void SetUptakePower(float power) { m_UptakePower = power; }

		void EnableIndexer(){
			SetBeltingPower(0.75);
			SetKickerPower(-1.0);
			SetUptakePower(1.0);
			if(tmr_BeaterBar.Get() > (m_BeaterBar ? 0.25 : 0.25)){ ToggleBeaterBar(); tmr_BeaterBar.Reset(); }
		}
		void DisableIndexer(){
			SetBeltingPower(0.0);
			SetKickerPower(0.0);
			SetUptakePower(0.0);
			RetractBeaterBar();
		}

		void ExtendHopper(){ m_Hopper = true; }
		void RetractHopper(){ m_Hopper = false; }
		void ToggleHopper(){ m_Hopper = !m_Hopper; }

		double GetSpeedFromDistance() { return (8807.7 + -85.625*vis_Boiler->GetDistance() +
				0.43217*vis_Boiler->GetDistance()*vis_Boiler->GetDistance()); }//400

		//(15 + 56.04*vis_Boiler->GetDistance() + -0.1053*vis_Boiler->GetDistance()*vis_Boiler->GetDistance())-400;

		//(-14647 + 293.88*vis_Boiler->GetDistance() + -1.0966*vis_Boiler->GetDistance()*vis_Boiler->GetDistance())+170;

		void SetTShooterPower(float tPower){ m_T_ShooterPower = tPower; }
		void SetBShooterPower(float bPower){ m_B_ShooterPower = bPower; }
		void SetShooterPower(float tPower, float bPower){ m_T_ShooterPower = tPower; m_B_ShooterPower = bPower; }
		double GetTShooterPower(){ return m_T_ShooterPower; }
		double GetBShooterPower(){ return m_B_ShooterPower; }

		void SetShooterSpeed(float speed){
			pid_T_ShooterSpeed->SetSetpoint(speed); pid_T_ShooterSpeed->Enable();
			pid_B_ShooterSpeed->SetSetpoint(speed); pid_B_ShooterSpeed->Enable();
		}

		double GetTShooterSpeed(){ return enc_T_ShooterSpeed->GetRate(); }
		double GetBShooterSpeed(){ return enc_B_ShooterSpeed->GetRate(); }

		bool IsTShooterPIDEnabled(){ return pid_T_ShooterSpeed->IsEnabled(); }
		bool IsBShooterPIDEnabled(){ return pid_B_ShooterSpeed->IsEnabled(); }
		bool IsShooterPIDEnabled(){ return pid_T_ShooterSpeed->IsEnabled() && pid_B_ShooterSpeed->IsEnabled(); }

		void SetTShooterPID(double P, double I, double D){ pid_T_ShooterSpeed->SetPID(P, I, D); }
		void SetBShooterPID(double P, double I, double D){ pid_B_ShooterSpeed->SetPID(P, I, D); }

		void DisableShooterPID(){ pid_T_ShooterSpeed->Disable(); pid_B_ShooterSpeed->Disable(); }
		void DisableShooter(){ DisableShooterPID(); SetShooterPower(0.0, 0.0); }

		void Update(){
			if(pid_T_ShooterSpeed->IsEnabled() || pid_B_ShooterSpeed->IsEnabled()){
				SetShooterPower(pid_T_ShooterSpeed->Get(), pid_B_ShooterSpeed->Get());
			}

			mtr_L_Belting->Set(m_L_Belting);
			mtr_R_Belting->Set(m_R_Belting);
			sol_Agitator->Set(m_BeaterBar);
			mtr_Kicker->Set(m_KickerPower);
			mtr_L_Uptake->Set(m_UptakePower);
			mtr_R_Uptake->Set(m_UptakePower);

			sol_Hopper->Set(m_Hopper);

			mtr_T_Shooter->Set(m_T_ShooterPower);
			mtr_B_Shooter_1->Set(m_B_ShooterPower);
			mtr_B_Shooter_2->Set(m_B_ShooterPower);
		}
	};
}
