import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
import os

import math
import copy
import modular
import random
import build_evolution
import motion
import mutate   

evo_times =5
basic_num =10
# ***Step 1. Initialize 5 robots, with 3 legs with different locations. 
# Create a dict "Survive", key is the file name, value is the fitness score
survive = dict()
num_species = 10
basic_module_num= 2
for i in range(num_species):
    mj,chassis = build_evolution.build_env()
    build_evolution.single_evolution(chassis, basic_module_num)
    tree = ET.ElementTree(mj)
    actuator = ET.SubElement(mj, "actuator")
    motors = dict()
    for j in range(basic_module_num):
        motors[f"velocity{j}"] = ET.SubElement(actuator, "velocity",joint = f"module__{j}slide_box_joint", kv = "1500")
    tree.write(f"actuator{i}.xml")
    survive[f"actuator{i}.xml"] = motion.fitness_cal(f"actuator{i}.xml",0)
#print(f"initialize:  {survive}")


# *** Step 2. Loop evo_times, in every loop, delete the lowest scores,  
for i in range(evo_times): 
    # Step 2.1:  Delete the lowest robos:
    
    keys_to_delete = sorted(survive, key=survive.get)[:5]
    print(f"keys_to_delete {keys_to_delete}")
    try:
        for key in keys_to_delete:
            os.remove(key)
            del survive[key]
        print(f"deleted, length {len(survive)} {survive}")
        
        # Step 2.2: Mutate the rest robos and keep the survived robos:
        sub_dict = dict()
        for robo in survive:
            key = mutate.mutate(survive, robo)
            #print(key)
            sub_dict[key] = motion.fitness_cal(key,0)
        survive = mutate.melt_dictionaries(survive, sub_dict)
    except:
        pass

for robo in survive:
    motion.fitness_cal(robo, 1)
