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

'''
Generate .h file from YAML files
NOTE: for int, this convertor generate const float
      for str, this convertor generate const int16_t based on dictionary.yaml
Usage: ./generate_all.py index
       where index is a yaml list
'''

# USER PARAMETERS START //

# output dir for the generated .h file
# if empty, use the same dir as the given YAML file
output_dir = "./"

# dictionary dir, convert str in YAML to int in .H
dict_dir = "./dictionary.yaml"

# USER PARAMETERS END  //

import argparse
from config_files.yaml_to_header_convertor import yaml_to_header

#parse args
parser = argparse.ArgumentParser(description='load controller config yaml file list')
parser.add_argument('yaml_list', type=str)
args = parser.parse_args()

with open(args.yaml_list, 'r') as yaml_list:
    lines = yaml_list.read().splitlines()
    for yaml_file in lines:
        yaml_to_header(yaml_file, output_dir, dict_dir)

