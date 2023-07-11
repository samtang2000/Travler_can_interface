# Program to calculate the helper parameters for Triangular Gaits
# from basic parameters

import math

height = float(input('Enter the step depth (m): '))
theta = float(input('Enter the sweep angle of the step (rad): '))
velocity = float(input('Enter the toe velocity of the step (m/s): '))
retraction_h = float(input('Enter the retraction distance (m): '))

# important
l_b = 2 * height * math.tan(theta / 2)
l_a = height * math.cos(theta / 2)


L_max = height * math.cos(theta / 2)
delta_L = L_max - retraction_h

period_b = l_b / velocity
total_period = 4 * period_b

print('{',height,',',theta,',',velocity,',',retraction_h, 
        ',', period_b, ',', total_period, ',', l_b, ',', delta_L, '}')

