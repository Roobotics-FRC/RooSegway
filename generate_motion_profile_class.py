#!/usr/bin/python

import sys

if (len(sys.argv) < 2):
  print('Usage: generate_motion_profile_class.py [csv file path] [name of class to generate]')
  sys.exit()

with open(sys.argv[1]) as csvFile:
    rawFile = csvFile.readlines()
    del rawFile[0]
    matrix = [[float(file_str) for file_str in line.replace('\n', '').split(',')] for line in rawFile]
    del matrix[0]

# matrix [duration(s), x, y, position, velocity, acceleration, jerk, heading]

tab = '    '

java_code = 'package org.usfirst.frc.team4373.robot.commands.profiles;\n\n'
java_code += 'public class ' + sys.argv[2] + ' extends MotionProfile {\n'
java_code += tab + 'public int numPoints = ' + str(len(matrix)) + ';\n'
java_code += tab + 'public double[][] points = new double[][]{\n'

for i in range(len(matrix)):
  java_code += (tab * 2) + '{' + str(matrix[i][3]) + ', ' + str(matrix[i][4]) + ', ' + str(matrix[i][0] * 1000) + '},\n'

java_code = java_code[:-2] + '\n'
java_code += tab + '};\n'
java_code += '}\n'

with open('src/main/java/org/usfirst/frc/team4373/robot/commands/profiles/' + sys.argv[2] + '.java', 'w') as output_file:
  output_file.write(java_code)
