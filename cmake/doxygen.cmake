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

# using doxygen to generate documents and using xdg-open to open index.html
find_program(DOXYGEN_EXE NAMES doxygen)
find_program(XDG_OPEN_EXE NAMES xdg-open)

# create document generating and viewing targets
if (DOXYGEN_EXE)
    # location of doxyfile and generated index.html
    set(DOXYFILE ${CMAKE_SOURCE_DIR}/Doxyfile)

    # generate documents
    add_custom_target(doc
        COMMAND ${DOXYGEN_EXE}
        DEPENDS ${DOXYFILE}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

endif()
