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

# find python3 executable
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# include all yaml files as dependence
file(GLOB_RECURSE YAML_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.yaml)

# read all yaml files from yaml_list
file(READ "yaml_list" yaml_list)

# use regular expression to split lines
string(REGEX MATCHALL "[A-Za-z0-9/_]*.yaml" yaml_file_list ${yaml_list})

# for debug
#message("${yaml_file_list}")

add_custom_target(yaml_headers
    COMMAND ${Python3_EXECUTABLE} generate_all.py yaml_list
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${YAML_FILES}
    )
