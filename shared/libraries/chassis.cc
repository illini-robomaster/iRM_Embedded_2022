//
// Created by neo on 12/28/21.
//

#include "chassis.h"
#include "can.h"
#include "bsp_error_handler.h"
#include "cmath"

extern CAN_HandleTypeDef hcan1;

namespace control{
    Chassis::Chassis(chassis_type_t robot_type) {
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
    void Chassis::Move(double x, double y, double z) {
	    switch (type) {
		    case HERO:
		    case STANDARD: {
			    // constexpr int16_t MAX_ABS_CURRENT = 12288;  // ~20A

//	    double moveSum = fabs(x) + fabs(y) + fabs(z);
//	    if (moveSum >= MAX_ABS_CURRENT) {
//		    int k = MAX_ABS_CURRENT/moveSum;
//		    x *= k;
//		    y *= k;`
//		    z *= k;
//	    }
			    double FL_speed = y + x + z;
			    double BL_speed = y - x + z;
			    double FR_speed = y - x - z;
			    double BR_speed = y + x - z;
			    motors[MOTOR_FL]->SetOutput(800);
			    motors[MOTOR_BL]->SetOutput((int16_t)(BL_speed+FL_speed));
			    motors[MOTOR_FR]->SetOutput((int16_t)FR_speed);
			    motors[MOTOR_BR]->SetOutput((int16_t)BR_speed);
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
