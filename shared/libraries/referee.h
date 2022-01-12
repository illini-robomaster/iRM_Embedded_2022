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

#pragma once

#include "bsp_error_handler.h"

namespace RoboMaster {
    constexpr int MAX_FRAME_LEN = 300;

/* Information From Referee */

/*
 * 0x0001 GAME_STATUS
 * 0x0002 GAME_RESULT
 * 0x0003 GAME_ROBOT_HP
 * 0x0005 ICRA_BUFF_DEBUFF_ZONE_STATUS [X]
 * 0x0101 EVENT_DATA
 * 0x0102 SUPPLY_PROJECTILE_ACTION
 * 0x0104 REFEREE_WARNING
 * 0x0105 DART_REMAINING_TIME
 * 0x0201 GAME_ROBOT_STATUS
 * 0x0202 POWER_HEAT_DATA
 * 0x0203 GAME_ROBOT_POS
 * 0x0204 BUFF
 * 0x0205 AERIAL_ROBOT_ENERGY
 * 0x0206 ROBOT_HURT
 * 0x0207 SHOOT_DATA
 * 0x0208 BULLET_REMAINING
 * 0x0209 RFID_STATUS
 * 0x020A DART_CLIENT_CMD
 * 0x0301 STUDENT_INTERACTIVE [X]
 * 0x0302 ROBOT_INTERACTIVE [x]
 * 0x0303 ROBOT_COMMAND [x]
 * 0x0304 ROBOT_COMMAND [x]
 * 0x0305 CLIENT_MAP_COMMAND [x]
 */

/* TODO(neo): above information with [x] should be implemented when needed in the future*/

typedef enum {
    GAME_STATUS = 0x0001,
    GAME_RESULT = 0x0002,
    GAME_ROBOT_HP = 0x003,
    EVENT_DATA = 0x0101,
    SUPPLY_PROJECTILE_ACTION = 0x0102,
    REFEREE_WARNING = 0x0104,
    DART_REMAINING_TIME = 0x0105,
    GAME_ROBOT_STATUS = 0x0201,
    POWER_HEAT_DATA = 0x0202,
    GAME_ROBOT_POS = 0x0203,
    BUFF = 0x0204,
    AERIAL_ROBOT_ENERGY = 0x0205,
    ROBOT_HURT = 0x0206,
    SHOOT_DATA = 0x0207,
    BULLET_REMAINING = 0x0208,
    RFID_STATUS = 0x0209,
    DART_CLIENT_CMD = 0x020A,
} referee_cmd;

/* ===== GAME_STATUS 0x0001 1Hz ===== */
typedef struct {
    uint8_t game_type:4;
    uint8_t game_progress:4;
    uint16_t stage_remain_time;

    uint64_t SyncTimeStamp;
} __packed game_status_t;

/* ===== GAME_RESULT 0x0002 ===== */
typedef struct {
    uint8_t winner;
} __packed game_result_t;

/* ===== GAME_ROBOT_HP 0x0003 1Hz ===== */
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_6_robot_HP;
    uint16_t red_7_robot_HP;

    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;

    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed game_robot_HP_t;

/* ===== EVENT_DATA 0x0101 1Hz ===== */
typedef struct {
    uint32_t event_type;
} __packed event_data_t;

/* ===== SUPPLY_PROJECTILE_ACTION 0x0102 ===== */
typedef struct {
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} __packed supply_projectile_action_t;

/* ===== REFEREE_WARNING 0x0104 ===== */
typedef struct {
    uint8_t level;
    uint8_t foul_robot_id;
} __packed referee_warning_t;

/* ===== DART_REMAINING_TIME 0x0105 1Hz ===== */
typedef struct {
    uint8_t dart_remaining_time;
} __packed dart_remaining_time_t;

/* ===== GAME_ROBOT_STATUS 0x0201 10Hz ===== */
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;

    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;

    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output:1;
    uint8_t mains_power_chassis_output:1;
    uint8_t mains_power_shooter_output:1;
} __packed game_robot_status_t;

/* ===== POWER_HEAT_DATA 0x0202 50Hz ===== */
typedef struct {
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} __packed power_heat_data_t;

/* ===== GAME_ROBOT_POS 0x0203 10Hz ===== */
typedef struct {
    float x;
    float y;
    float z;
    float yaw;
} __packed game_robot_pos_t;

/* ===== BUFF 0x0204 1Hz ===== */
typedef struct {
    uint8_t power_rune_buff;
} __packed buff_t;

/* ===== AERIAL_ROBOT_ENERGY 0x0205 10Hz ===== */
typedef struct {
    uint8_t attack_time;
} __packed aerial_robot_energy_t;

/* ===== ROBOT_HURT 0x0206 ===== */
typedef struct {
    uint8_t armor_id:4;
    uint8_t hurt_type:4;
} __packed robot_hurt_t;

/* ===== SHOOT_DATA 0x0207 ===== */
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} __packed shoot_data_t;

/* ===== BULLET_REMAINING 0x0208 10Hz ===== */
typedef struct {
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} __packed bullet_remaining_t;

/* ===== RFID_STATUS 0x0209 1Hz ===== */
typedef struct {
    uint32_t rfid_status;
} __packed rfid_status_t;

/* ===== DART_CLIENT_CMD 0x020A 10Hz ===== */
typedef struct {
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} __packed dart_client_cmd_t;

class Referee {
public:
    game_status_t game_status;
    game_result_t game_result;
    game_robot_HP_t game_robot_HP;
    event_data_t event_data;
    supply_projectile_action_t supply_projectile_action;
    referee_warning_t referee_warning;
    dart_remaining_time_t dart_remaining_time;
    game_robot_status_t game_robot_status;
    power_heat_data_t power_heat_data;
    game_robot_pos_t game_robot_pos;
    buff_t buff;
    aerial_robot_energy_t aerial_robot_energy;
    robot_hurt_t robot_hurt;
    shoot_data_t shoot_data;
    bullet_remaining_t bullet_remaining;
    rfid_status_t rfid_status;
    dart_client_cmd_t dart_client_cmd;

    bool Update(const uint8_t* data, int length);

private:
    uint8_t buffer[MAX_FRAME_LEN];
    bool CheckHeader(const uint8_t* data, int length);
    bool CheckFrame(const uint8_t* data, int length);
    bool ProcessData(int cmd_id, const uint8_t* data, int length);
};

}
