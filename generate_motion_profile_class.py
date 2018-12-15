#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# This script converts a motion profile CSV from https://github.com/vannaka/Motion_Profile_Generator to a Java class
#

__author__ = "aaplmath"
__version__ = "0.1"

import sys

if (len(sys.argv) != 3):
  print('Usage: generate_motion_profile_class.py [csv file path] [name of class to generate]')
  sys.exit()

with open(sys.argv[1]) as csvFile:
    rawFile = csvFile.readlines()
    del rawFile[0]
    matrix = [[float(file_str) for file_str in line.replace('\n', '').split(',')] for line in rawFile]
    del matrix[0]

# matrix elements are of the format [duration(s), x, y, position, velocity, acceleration, jerk, heading]

tab = '    '

java_code = 'package org.usfirst.frc.team4373.robot.commands.profiles;\n\n'
java_code += 'public class ' + sys.argv[2] + ' implements MotionProfile {\n'

java_code += tab + 'private double[][] points = new double[][]{\n'
for i in range(len(matrix)):
  java_code += (tab * 2) + '{' + str(matrix[i][3]) + ', ' + str(matrix[i][4]) + ', ' + str(matrix[i][0] * 1000) + '},\n'
java_code = java_code[:-2] + '\n'
java_code += tab + '};\n\n'

java_code += tab + '/**\n'
java_code += tab + ' * Gets the number of points in the profile.\n'
java_code += tab + ' * @return the number of points in the profile.\n'
java_code += tab + ' */\n'
java_code += tab + 'public int getNumPoints() {\n' + (tab * 2) + 'return ' + str(len(matrix)) + ';\n' + tab + '}\n\n'

java_code += tab + '/**\n'
java_code += tab + ' * Gets the points in the profile.\n'
java_code += tab + ' * @return the points in the profile.\n'
java_code += tab + ' */\n'
java_code += tab + 'public double[][] getPoints() {\n'
java_code += (tab * 2) + 'return points;\n'
java_code += tab + '}\n'

java_code += '}\n'

with open('src/main/java/org/usfirst/frc/team4373/robot/commands/profiles/' + sys.argv[2] + '.java', 'w') as output_file:
  output_file.write(java_code)
