#!/usr/bin/env python3

'''
Generate .h file from YAML files
NOTE: for int, this convertor generate const float
'''

# if empty, use the same dir as the given YAML file
target_dir = ""

import yaml
import argparse

#parse args
parser = argparse.ArgumentParser(description='load controller config yaml file')
parser.add_argument('file', type=str)
args = parser.parse_args()

with open(args.file, 'r') as file:
    docs = yaml.safe_load(file)
    
    # discard the .yaml part from the YAML file
    h_file_name = args.file.split("/")[-1].split(".")
    if len(h_file_name) == 1:
        h_file_name = h_file_name[0]
    else:
        h_file_name = "".join(h_file_name[:-1])
    
    # generate file name and dir for the .H file
    h_file_name = h_file_name.upper() + "_CONFIG.h"
    if target_dir == "":
        target_dir = args.file[:-len(args.file.split("/")[-1])]
    
    # write the generated .h file. NOTE: this is not append
    f = open(target_dir + h_file_name, 'w')

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
        if type(node) == float or type(node) == int:
            f.write("const float _" + name + " = " + str(node) + ";\n")
            return
        elif type(node) == str:
            # TODO translator
            return
        # Recursion
        for i in node:
            dfs_yaml(node[i], name + "_" + i.upper())
    
    # call DFS to generate all variables in YAML
    dfs_yaml(docs, "")
    
    # extra newline as C convention
    f.write("\n")
    
    f.close()

