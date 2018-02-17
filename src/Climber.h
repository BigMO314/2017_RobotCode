#pragma once
#include "WPILib.h"
#include "Robot Specifications.h"

namespace Robot{
	class Climber{
	private:
		WPILib::Victor* mtr_L_Climber;
		WPILib::Victor* mtr_R_Climber;

		float m_Power = 0;

	public:
		Climber(Victor* ref_L_Climber, Victor* ref_R_Climber){
			this->mtr_L_Climber = ref_L_Climber;
			this->mtr_R_Climber = ref_R_Climber;
		}
		void Enable() { m_Power = 1.0; }
		void Disable() { m_Power = 0.0; }
		void SetPower(float power) { m_Power = power; }

		void Update(){
			mtr_L_Climber->Set(m_Power);
			mtr_R_Climber->Set(m_Power);
		}
	};
}
