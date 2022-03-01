# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2021                                                    #
#  Illini RoboMaster @ University of Illinois at Urbana-Champaign.       #
#                                                                        #
#  This program is free software: you can redistribute it and/or modify  #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation, either version 3 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
#  You should have received a copy of the GNU General Public License     #
#  along with this program. If not, see <http://www.gnu.org/licenses/>.  #
#                                                                        #
# ---------------------------------------------------------------------- #

project(example_gimbal ASM C CXX)

irm_add_arm_executable(${PROJECT_NAME}_scout2022
    TARGET DJI_Board_TypeA
    SOURCES scout_2022.cc)

irm_add_arm_executable(${PROJECT_NAME}_hero2022
    TARGET DJI_Board_TypeA
    SOURCES hero_2022.cc)
    
irm_add_arm_executable(${PROJECT_NAME}_imu_scout2022
    TARGET DJI_Board_TypeA
    SOURCES imu_scout_2022.cc)
    
irm_add_arm_executable(${PROJECT_NAME}_imu_hero2022
    TARGET DJI_Board_TypeA
    SOURCES imu_hero_2022.cc)
