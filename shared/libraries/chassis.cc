/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "chassis.h"
#include "can.h"
#include "bsp_error_handler.h"
 #include "cmath"

extern CAN_HandleTypeDef hcan1;

namespace control{
    Chassis::Chassis(robot_type_t robot_type, const int* motor_id, const float* pid):
    pid_FL(PIDController(pid[0], pid[1], pid[2])),
    pid_BL(PIDController(pid[0], pid[1], pid[2])),
    pid_FR(PIDController(pid[0], pid[1], pid[2])),
    pid_BR(PIDController(pid[0], pid[1], pid[2])) {
	    type = robot_type;
	    switch (type) {
		    case HERO:
		    // the same as STANDARD
		    case STANDARD: {
			    motors = new MotorCANBase *[4];
			    auto* can1 = new bsp::CAN(&hcan1, 0x201);
			    motors[MOTOR_FL] = new Motor3508(can1, 0x200 + motor_id[0]);
			    motors[MOTOR_BL] = new Motor3508(can1, 0x200 + motor_id[1]);
			    motors[MOTOR_FR] = new Motor3508(can1, 0x200 + motor_id[2]);
			    motors[MOTOR_BR] = new Motor3508(can1, 0x200 + motor_id[3]);
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
	    constexpr int MAX_ABS_CURRENT = 12288;  // ~20A
	    constexpr int MAX_ABS_REMOTE = 32767;
	    switch (type) {
		    case HERO:
		    case STANDARD: {
			    x = x * MAX_ABS_CURRENT / MAX_ABS_REMOTE;
			    y = y * MAX_ABS_CURRENT / MAX_ABS_REMOTE;
			    z = z * MAX_ABS_CURRENT / MAX_ABS_REMOTE;
			    float moveSum = fabs(x) + fabs(y) + fabs(z);
			    if (moveSum >= MAX_ABS_CURRENT) {
				    float k = MAX_ABS_CURRENT / moveSum;
				    x *= k;
				    y *= k;
				    z *= k;
			    }
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
