import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET

import math
import copy
import modular
import random

#--** Motion Control ** -----------------------------------------------
def fitness_cal(file_name, view):
    m = dm_control.mujoco.MjModel.from_xml_path(file_name)
    d = dm_control.mujoco.MjData(m)
    viewer = mujoco_viewer.MujocoViewer(m, d)

    motors = m.nu
    step = 5
    duration_per_direction = 50
    n = 1000
    pos=[]

    for i in range(n): 
        if i ==0:
            pos.append(copy.copy(d.body(f'module__0module-top').xpos))
        if i == (n-1):
            pos.append(copy.copy(d.body(f'module__0module-top').xpos))

        # ** All joints move back and forth at the same time *****---------------------------------
        current_step = i // duration_per_direction
        if current_step%2 == 0 :
            d.ctrl[:motors] = step*100 
            mujoco.mj_step(m, d)
        else:
            d.ctrl[:motors] = -step*100 
            mujoco.mj_step(m, d)

        # ** Sin Wave Control --------------------------------------------------------------
        # amplitude = 500  # Amplitude of sine wave
        # frequency = 0.08  # Frequency of sine wave
        # for motor_id in range(motors):
        #     phase_shift = motor_id * np.pi / motors  # Different phase shift for each motor
        #     d.ctrl[motor_id] = amplitude * np.sin(frequency * i + phase_shift)
        # mujoco.mj_step(m, d)
        # *******--------------------------------------------------------------

        if view == 1:
            viewer.render()
        pass
        #fitness function = the Euclidean Distance / time
    fitness = math.sqrt((pos[0][0]-pos[1][0])**2 + (pos[0][1]-pos[1][1])**2) / n
    return fitness

#fitness_cal("actuator0.xml")s
    # current_step = i // duration_per_direction
    # if viewer.is_alive:
    #     if current_step%2 == 0 :
    #         d.ctrl[:motors] = step*100 
    #         mujoco.mj_step(m, d)
    #     else:
    #         d.ctrl[:motors] = -step*100 
    #         mujoco.mj_step(m, d)
    #     viewer.render()
    # else:
    #     break
# --------------fitness calculation -------------------------------------------