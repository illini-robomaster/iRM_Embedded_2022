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

import yaml

##
# @description: Convert YAML to C/CPP header. 
# @params: yaml_dir, YAML file that need to be converted
#          output_dir, output directory of the header generated
#          dict_dir, dictionary to convert str in YAML to int16_t in header
# @note: for int, this convertor generate const float
#        for str, this convertor generate const int16_t based on dictionary.yaml
##
def yaml_to_header(yaml_dir, output_dir, dict_dir = ""):
    # read YAML file
    yaml_file = open(yaml_dir, "r")
    yaml_file = yaml.safe_load(yaml_file)
    
    # load dictionary
    if dict_dir != "":
        dict_dir = dict_dir
        dict_file = open(dict_dir, "r")
        dictionary = yaml.safe_load(dict_file)
    
    # set generated header name
    if yaml_file != None and "header_name" in yaml_file and yaml_file["header_name"] != None:
        # if name defined in YAML file
        h_file_name = yaml_file["header_name"]
    else:
        # else, use the YAML file name
        h_file_name = yaml_dir.split("/")[-1].split(".")
        if len(h_file_name) == 1:
            h_file_name = h_file_name[0]
        else:
            h_file_name = "".join(h_file_name[:-1])
    
    # generate file name and dir for the .H file
    h_file_name = h_file_name.upper() + "_CONFIG.h"
    if output_dir == "":
        # discard the .yaml part from the YAML file to get the dir
        output_dir = yaml_dir[:-len(yaml_dir.split("/")[-1])]
    
    print("Generating " + h_file_name + "...")

        
    # write the generated .h file. NOTE: this is not append
    f = open(output_dir + h_file_name, 'w')

    # disclaimer
    f.write('''/****************************************************************************
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

/********* THIS FILE IS AUTO GENERATED. DO NOT CHANGE IT MANUALLY. **********/
/********* CHANGE CONFIG FILES IN /config TO MODIFY THIS FILE. **************/
/********* EVERY GENERATION REWRITE THE WHOLE FILE. *************************/

'''
    )
    
    
    # DFS generate all variables for .H file
    def dfs_yaml(node, name):
        # Base Cases
        if node == None:
            return
        if type(node) == float or type(node) == int:
            f.write("const float _" + name + " = " + str(node) + ";\n")
            return
        elif type(node) == str:
            f.write("const int16_t _" + name + " = " + str(dictionary[node]) + ";\n")
            return
        # Recursion
        for i in node:
            # ignore "header_name"
            if i != "header_name":
                dfs_yaml(node[i], name + "_" + i.upper())
    
    # call DFS to generate all variables in YAML
    dfs_yaml(yaml_file, "")
    
    # extra newline as C convention
    f.write("\n")
        
    dict_file.close()
    f.close()

