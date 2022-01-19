#!/usr/bin/env python3

import yaml
import argparse

parser = argparse.ArgumentParser(description='load controller config yaml file')
parser.add_argument('file', type=str)
args = parser.parse_args()

print(args.file)

with open(args.file, 'r') as file:
    docs = yaml.safe_load(file)
    print(docs['controller']['p1'])
    print(docs['controller']['p2'])
