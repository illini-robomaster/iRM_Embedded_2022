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


file(GLOB_RECURSE BSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cc)
file(GLOB_RECURSE MODULE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/module/*.cc)
file(GLOB_RECURSE ROLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/role/*.cc)
file(GLOB_RECURSE TASK_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/task/*.cc)


set(IRM_SOURCES ${BSP_SOURCES} ${MODULE_SOURCES} ${ROLE_SOURCES} ${TASK_SOURCES})
set(IRM_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/module
    ${CMAKE_CURRENT_SOURCE_DIR}/role
    ${CMAKE_CURRENT_SOURCE_DIR}/task
    )

add_subdirectory(example)    

irm_add_board_specific_library(DJI_Board_TypeA_irm
    TARGET DJI_Board_TypeA
    SOURCES ${IRM_SOURCES}
    INCLUDES ${IRM_INCLUDES})

