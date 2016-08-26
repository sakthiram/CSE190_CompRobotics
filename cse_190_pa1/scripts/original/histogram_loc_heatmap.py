import random 
from math import *
# The function localize takes the following arguments:
#
# heatmap:
#        2D list, each entry is a temperature value
#
# texturemap:
#        2D list, each entry is either 'R' (rough) or 'S' (smooth) surface

#
# temp_measurements:
#        list of temperature measurements taken by the robot, each entry is a
#        floating point value which is the true temperature with additive
#        Gaussian noise
#
# texture_measurements:
#        list of texture measurements taken by the robot, each entry is either a
#        'R' or 'S'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        texture measurement is correct; the probability that the measurement is
#        incorrect is 1- p_correct_texture_sensor
#
# std_temp_meas:
#        the standard deviation in the temperature measurements, which have
#        additive Gaussian noise

#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should RETURN (not just show or print) a 2D list (of the same
# dimensions as the heat_map and texture_map) that gives the probabilities that
# the robot occupies each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up





# we should have two possible motion models, one is given below
# For the other model the robot moves in the correct direction with probability p_move
# however, with probability (1-p_move)/4 it moves in one of the (three) incorrect
# directions or doesn't move at all. For the second model the implementation
# of move given below will have to be modified.
  

def move (p_curr, motion, p_move):
    """ This function updates the belief of the robot's position based on the
        movement command and motion model"""
    q = [[ x for x in row] for row in p_curr] 
    #show(p_curr)
    
    num_cols = len(p_curr[0])
    num_rows = len(p_curr)
    for i in range(num_rows):
        for j in range(num_cols):
            #print i, j
            from_posx = (i - motion[0] )% num_rows
            from_posy = (j - motion[1] )% num_cols
            q[i][j] = p_move * p_curr[from_posx][from_posy] + (1-p_move) * p_curr[i][j]

    alpha = 0.0        
    for i in range(len(q)):        
       alpha +=sum(q[i])

    if alpha ==0:
        print "sum of probabilities is zero"
    else:
        q = [[ x/alpha for x in row] for row in q] 
 
    return q

def sense_texture(p_curr, sensor_data, p_correct, texture_map):
    """ Returns the updated the belief of the robot's pose after fusing in one
        texture measurement. The updated belief is computed using the texture_map
        of the world, the sensor data and the probability that the sensor
        measurement is correct"""
    q = p_curr
    num_cols = len(p_curr[0])
    num_rows = len(p_curr)
    for i in range(num_rows):
        for j in range(num_cols):
            hit= texture_map[i][j]==sensor_data
            q[i][j] = (p_correct * hit + (1-p_correct) * (1-hit)) *p_curr[i][j]
       
    alpha = 0.0        
    for i in range(len(q)):        
       alpha +=sum(q[i])

    if alpha ==0:
        print "sum of probabilities is zero"
    else:
        q = [[ x/alpha for x in row] for row in q] 
 
    return q


def sense_temperature (p_curr, sensor_data, std_temp_meas, heat_map):
    """ Returns the updated the belief of the robot's pose after fusing in one
        temperature measurement. The updated belief is computed using the texture_map
        of the world, the sensor data and the probability that the sensor measurement
        is correct"""
    
      
    q = [[0.0 for x in row] for row in p_curr]
    num_cols = len(p_curr[0])
    num_rows = len(p_curr)
    for i in range(num_rows):
        for j in range(num_cols):
            if std_temp_meas ==0:
                p_update = heat_map[i][j]==sensor_data
                
            else:
                p_update = 1/(sqrt(2*pi)*std_temp_meas)*e**(-0.5*(float(heat_map[i][j]-sensor_data)/std_temp_meas)**2)

            q[i][j] = p_update * p_curr[i][j]
       
    alpha = 0.0        
    for i in range(len(q)):        
       alpha +=sum(q[i])

    if alpha ==0:
        print "sum of probabilities is zero"
    else:
        q = [[ x/alpha for x in row] for row in q] 
 
    return q

def localize(texture_map,measurements,motions,sensor_right,p_move,
             heat_map, temp_measurements, std_temp_meas, use_temp_meas, use_texture_meas):
    """ The main localization function, returns the belief of the robot pose
        after fusing all the meaurements and movement commands into the initial belief"""
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / float(len(texture_map)) / float(len(texture_map[0]))
    p = [[pinit for row in range(len(texture_map[0]))] for col in range(len(texture_map))]
    
    for i in range(len(measurements)):
      
        p = move (p, motions[i], p_move)
        if use_texture_meas:
            p = sense_texture(p, measurements[i], sensor_right, texture_map)
        if use_temp_meas:
            p = sense_temperature( p, temp_measurements[i], std_temp_meas, heat_map)
     
    return p

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'
    

def generate_heatmap(pipe_map):
    """ Generates a heat map based on whether there is a cold or hot pipe running
        under each tile. Tiles with no pipes underneath will have a "warm" temperature """
    temp_cold = 20.0
    temp_hot  = 40.0
    temp_warm = 25.0
    heat_map = [[ 0.0 for x in row] for row in pipe_map]
    for i in range(len(pipe_map)):
        for j in range(len(pipe_map[0])):
            if pipe_map[i][j] == 'C':
                heat_map[i][j] = temp_cold

            elif pipe_map[i][j] == 'H':
                heat_map[i][j] = temp_hot
            else:
                heat_map[i][j] = temp_warm

    return heat_map

def add_noise( true_val, std_noise):
    """ Returns temperature measurements after adding Gaussian noise"""
    meas = []
    for i in range(len(true_val)):
        meas.append( true_val[i] + ceil(random.gauss(0,std_noise)*100)/100)
   
    return meas

## Inputs to localization: 
# configuration parameters, these should be configurable per instance of simulation in ROS
sensor_right = .99
p_move = 1
std_temp_meas = 10

### end of configuration parameters
# pipe_map shows what kind of pipe is running under each tile
# 'C': cold, '-': no pipe, 'H': hot
pipe_map = [['C','-','H','H','-'],
            ['C','-','H','-','-'],
            ['C','-','H','-','-'],
            ['C','C','H','H','H']]

# texture_map contains the texture of each tile, that may be either
# 'R': rough or 'S': smooth

texture_map = [['S','S','S','S','R'],
               ['R','R','S','R','R'],
               ['R','S','S','S','S'],
               ['S','R','R','S','R']]



heat_map = generate_heatmap(pipe_map)
#show(heat_map)

motions = [[0,0],[0,1],[1,0],[1,0],[0,1],[0,1]]

def generate_measurements(seed, motions, heat_map, texture_map, sensor_right, std_temp_meas):

    random.seed(seed)
    num_rows = len(heat_map)
    num_cols = len(heat_map[0])

    path_taken = [['-' for x in row] for row in heat_map]
    curr_loc = [random.randint(0, num_rows-1), random.randint(0, num_cols -1)]

    texture_measurements = ['R' for x in range(len(motions))] 
    temp_measurements   = [ 20.0 for x in range(len(motions))] 

    print "Starting location of robot is", curr_loc, "(unknown to robot)"



    for i in range(len(motions)):
    # In ROS the motion of the robot should be simulated using the assumed model, but here we are not taking
    # the probabilities into account
        curr_loc = [(curr_loc[0]+motions[i][0]) % num_rows , (curr_loc[1]+motions[i][1]) % num_cols]
        path_taken[curr_loc[0]][curr_loc[1]]=str(i)
        # select the texture measurement based on the probability of getting a correct value (sense_right)
        p = random.uniform(0,1)
        if p < sensor_right:
            texture_measurements[i]= texture_map[curr_loc[0]][curr_loc[1]]
        else:
            if texture_map[curr_loc[0]][curr_loc[1]] =='R':
                texture_measurements[i]='S'
            else:
                texture_measurements[i]='R'
        temp_measurements[i] = heat_map[curr_loc[0]][curr_loc[1]]
        
    temp_measurements = add_noise(temp_measurements, std_temp_meas)
    print "Path taken by robot denoted by numbers"
    for i in range(len(path_taken)):
        print path_taken[i]
    #print "Temp measurements",temp_measurements
    return (temp_measurements, texture_measurements)



#### End of inputs to localization
(temp_measurements, texture_measurements)= generate_measurements(0, motions, heat_map, texture_map, sensor_right, std_temp_meas)
p = localize(texture_map,texture_measurements, motions, sensor_right, p_move,
             heat_map, temp_measurements, std_temp_meas, use_temp_meas=1, use_texture_meas=1)
print "Final posterior distribution"
show(p) # displays your answer
