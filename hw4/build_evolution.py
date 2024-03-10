import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET

import math
import copy
import modular
import random
#--** Define Environment & Plane Textures **---------------------------------------------------------
def build_env():
    mj = ET.Element("mujoco")
    option = ET.SubElement(mj, "option", gravity = "0 0 -9.81" )
    worldbody = ET.SubElement(mj,"worldbody")
    light = ET.SubElement(worldbody, "light", diffuse="0.8 0.8 0.8", pos="0 0 3", dir="0 0 -1")
    asset = ET.SubElement(mj, "asset")
    textures = [
        {"type": "skybox", "builtin": "gradient", "rgb1": ".3 .5 .7", "rgb2": "0 0 0", "width": "512", "height": "512"},
        {"name": "body", "type": "cube", "builtin": "flat", "mark": "cross", "width": "128", "height": "128",
        "rgb1": "0.8 0.6 0.4", "rgb2": "0.8 0.6 0.4", "markrgb": "1 1 1", "random": "0.01"},
        {"name": "grid", "type": "2d", "builtin": "checker", "width": "512", "height": "512", "rgb1": ".1 .2 .3",
        "rgb2": ".2 .3 .4"}
    ]
    materials = [
        {"name": "body", "texture": "body", "texuniform": "true", "rgba": "0.8 0.6 .4 1"},
        {"name": "grid", "texture": "grid", "texrepeat": "1 1", "texuniform": "true", "reflectance": ".2"}
    ]
    # Add texture elements
    for tex in textures:
        ET.SubElement(asset, "texture", tex)
    # Add material elements
    for mat in materials:
        ET.SubElement(asset, "material", mat)
    plane = ET.SubElement(worldbody, "geom", size="100 100 .05",type="plane", material="grid",condim="3")   
    chassis = ET.SubElement(worldbody,"body", name = "chassis" ,pos = "0 2 3")
    chassis_joint = ET.SubElement(chassis, "joint", type = "free")
    chassis_geom = ET.SubElement(chassis,"geom", type = "box", size=".1 3 3", rgba = ".5 .5 .5 1" ,mass="800")

    return mj,chassis
# inertial = ET.SubElement(chassis, "inertial", mass=1)
#--** Building Units** ------------------------------------------------------------

# module1 = modular.build_unit(chassis,"module1", "1 -5 -2")
# modular.build_unit(module1,"module2", "0 0 0")


#--** Evolutionary Iterations ** -----------------------------------------------

def single_evolution(chassis, evo_num):
    y_rand1 = random.randint(-5,5)
    z_rand1 = random.randint(-2,2)
    # build the first module
    modules= dict()
    modules["module__0"] = modular.build_unit(chassis,"module__0", f"1 {y_rand1} {z_rand1}")
    ns = []
    for i in range(1,evo_num):
        # add some randomness
        if random.randint(1,4)%2 == 1:
            y_rand = random.randint(-5,5)
            z_rand = random.randint(-2,2)
            # build legs in from chassis
            modules[f"module__{i}"] = modular.build_unit(chassis,f"module__{i}", f"0 {y_rand} {z_rand}")
        else:
            # extend legs
            #print(f"modules{modules}")
            if i>1:
                n = random.randint(1,i-1)
                while n in ns:
                    n = random.randint(1,i-1)
                ns.append(n)
            else:
                n = 0
            modules[f"module__{i}"] = modular.build_unit(modules[f"module__{n}"],f"module__{i}", "0 0 0")



