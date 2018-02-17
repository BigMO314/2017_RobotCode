#pragma once
#include "MOlib.h"

namespace Robot{
	class GearManagement{
	public:
		GearManagement(WPILib::Victor* ref_Roller, WPILib::Solenoid* ref_Intake, WPILib::DigitalInput* ref_IntakeLoaded){
			this->mtr_Roller = ref_Roller;
			this->sol_Intake = ref_Intake;
			this->pho_IntakeLoaded = ref_IntakeLoaded;
		}

		enum class IntakeState{ kOut, kIn };
		void ExtendIntake(){ m_IntakeState = IntakeState::kOut; }
		void RetractIntake(){ m_IntakeState = IntakeState::kIn; }
		void ToggleIntake(){ (m_IntakeState == IntakeState::kOut) ? m_IntakeState = IntakeState::kIn : m_IntakeState = IntakeState::kOut; }
		IntakeState GetIntakeState(){ return m_IntakeState; }

		void SetIntakePower(float power){ m_RollerPower = power; }
		void EnableIntakeIn(){ SetIntakePower(1.0);}
		void EnableIntakeOut(){ SetIntakePower(-1.0); }
		void DisableIntake(){ SetIntakePower(0.0); }
		double GetIntakePower(){ return m_RollerPower; }

		void Update(){
			sol_Intake->Set(m_IntakeState == IntakeState::kOut);
			mtr_Roller->Set(m_RollerPower);
		}
		bool GetGearLoaded(){ return pho_IntakeLoaded->Get(); }

	private:
		//Defineing the gear Solenoids.
		WPILib::Victor*					mtr_Roller;
		WPILib::Solenoid*				sol_Intake;
		WPILib::DigitalInput* 			pho_IntakeLoaded;

		IntakeState m_IntakeState	= IntakeState::kIn;
		float m_RollerPower			= 0.0;
	};
}
