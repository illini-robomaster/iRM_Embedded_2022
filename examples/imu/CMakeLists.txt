# ---------------------------------------------------------------------- #
#                                                                        #
#  Copyright (C) 2022                                                    #
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

project(example_imu ASM C CXX)

irm_add_arm_executable(${PROJECT_NAME}_typeA
    TARGET DJI_Board_TypeA
    SOURCES typeA.cc)

irm_add_arm_executable(imu_estimate
    TARGET DJI_Board_TypeA
    SOURCES poseEstimate.cc)

irm_add_arm_executable(${PROJECT_NAME}_typeC
    TARGET DJI_Board_TypeC
    SOURCES typeC.cc)

irm_add_arm_executable(ist8310_timestamp
    TARGET DJI_Board_TypeC
    SOURCES ist8310_timestamp.cc)

irm_add_arm_executable(imu_ahrs
    TARGET DJI_Board_TypeC
    SOURCES ahrs.cc)
