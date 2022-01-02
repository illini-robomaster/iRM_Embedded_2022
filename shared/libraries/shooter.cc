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

#include "shooter.h"
#include "can.h"
#include "bsp_error_handler.h"

extern CAN_HandleTypeDef hcan1;

namespace control {
    Shooter::Shooter(robot_type_t robot_type, const int* motor_id, const float* fire_pid, const float* load_pid):
    pid_Left(PIDController(fire_pid[0], fire_pid[1], fire_pid[2])),
    pid_Right(PIDController(fire_pid[0], fire_pid[1], fire_pid[2])),
    pid_Bullet(PIDController(load_pid[0], load_pid[1], load_pid[2])){
	    type = robot_type;
	    motors = new MotorCANBase *[3];
	    auto* can1 = new bsp::CAN(&hcan1, 0x201);
	    motors[MOTOR_Left] = new Motor3508(can1, 0x200 + motor_id[0]);
	    motors[MOTOR_Rright] = new Motor3508(can1, 0x200 + motor_id[1]);
	    switch (type) {
		    case HERO:
			    motors[MOTOR_Bullet] = new Motor3508(can1, 0x200 + motor_id[2]);
			    break;
		    case STANDARD:
			    motors[MOTOR_Bullet] = new Motor2006(can1, 0x200 + motor_id[2]);
			    break;
		    default:
			    RM_EXPECT_TRUE(false, "not supported robot type");
	    }
    }
    void Shooter::Fire(float speed) {
//	    constexpr int MAX_ABS_CURRENT = 12288;  // ~20A
//	    constexpr int MAX_ABS_REMOTE = 32767;
//	    speed = speed * MAX_ABS_CURRENT / MAX_ABS_REMOTE;
	    motors[MOTOR_Left]->SetOutput(pid_Left.ComputeOutput(motors[MOTOR_Left]->GetOmegaDelta( speed)));
	    motors[MOTOR_Rright]->SetOutput(pid_Right.ComputeOutput(motors[MOTOR_Rright]->GetOmegaDelta(-1 * speed)));
	    control::MotorCANBase::TransmitOutput(motors, 2);
    }
    void Shooter::Load(float speed) {
	    switch (type) {
		    case HERO:
			    speed = -1 * speed / 10;
			    break;
		    case STANDARD:
			    break;
		    default:
			    RM_EXPECT_TRUE(false, "not supported robot type");
	    }
	    motors[MOTOR_Bullet]->SetOutput(pid_Bullet.ComputeOutput(motors[MOTOR_Bullet]->GetOmegaDelta(speed)));
	    control::MotorCANBase::TransmitOutput(&motors[2], 1);
    }
}
