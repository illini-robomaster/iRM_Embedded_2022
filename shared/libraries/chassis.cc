//
// Created by neo on 12/28/21.
//

#include "chassis.h"
#include "can.h"
#include "bsp_error_handler.h"
 #include "cmath"

extern CAN_HandleTypeDef hcan1;

namespace control{
    Chassis::Chassis(chassis_type_t robot_type, float kp, float ki, float kd): pid_FL(PIDController(kp, ki, kd)), pid_BL(PIDController(kp, ki, kd)), pid_FR(PIDController(kp, ki, kd)), pid_BR(PIDController(kp, ki, kd)) {
	    type = robot_type;
	    switch (type) {
		    case HERO:
		    case STANDARD: {
			    motors = new MotorCANBase *[4];
			    auto* can1 = new bsp::CAN(&hcan1, 0x201);
			    motors[MOTOR_FL] = new Motor3508(can1, 0x202);
			    motors[MOTOR_BL] = new Motor3508(can1, 0x203);
			    motors[MOTOR_FR] = new Motor3508(can1, 0x201);
			    motors[MOTOR_BR] = new Motor3508(can1, 0x204);
			    break;
		    }
		    case ENGINEER: {
			    // TODO: chassis engineer not implemented
			    break;
		    }
		    default:
			    RM_EXPECT_TRUE(false, "not supported robot type");
	    }
    }
    void Chassis::Move(float x, float y, float z) {
	    switch (type) {
		    case HERO:
		    case STANDARD: {
			    x = x * 12288 / 32767;
			    y = y * 12288 / 32767;
			    z = z * 12288 / 32767;
			    // constexpr int16_t MAX_ABS_CURRENT = 12288;  // ~20A
//			    double moveSum = fabs(x) + fabs(y) + fabs(z);
//			    if (moveSum >= MAX_ABS_CURRENT) {
//				    int k = MAX_ABS_CURRENT/moveSum;
//				    x *= k;
//				    y *= k;
//				    z *= k;
//			    }
			    float FL_speed = y + x + z;
			    float BL_speed = y - x + z;
			    float FR_speed = -1 * (y - x - z);
			    float BR_speed = -1 * (y + x - z);
			    motors[MOTOR_FL]->SetOutput(pid_FL.ComputeOutput(motors[MOTOR_FL]->GetOmegaDelta(FL_speed)));
			    motors[MOTOR_BL]->SetOutput(pid_BL.ComputeOutput(motors[MOTOR_BL]->GetOmegaDelta(BL_speed)));
			    motors[MOTOR_FR]->SetOutput(pid_FR.ComputeOutput(motors[MOTOR_FR]->GetOmegaDelta(FR_speed)));
			    motors[MOTOR_BR]->SetOutput(pid_BR.ComputeOutput(motors[MOTOR_BR]->GetOmegaDelta(BR_speed)));
			    control::MotorCANBase::TransmitOutput(motors, 4);
		    }
		    case ENGINEER:
			    // TODO: chassis engineer not implemented
			    break;
		    default:
			    RM_EXPECT_TRUE(false, "not supported robot type");
	    }
    }
}
