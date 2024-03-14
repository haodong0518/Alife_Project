import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET

import math
import copy
import modular
import random
import build_evolution
import motion

#--** Find the leaf node ** -----------------------------------------------
def find_leaf_nodes(root , leaves):
    """
    Recursively finds all leaf nodes in an XML tree starting from 'element'.
    'leaves' is a list where leaf nodes will be added.
    """
    # Check if this element is a leaf node (has no children)
    try:
        if list(root)[-1].tag != 'body':
            #print(root.attrib)
            #print(list(root))
            leaves.append(root)
        else:
            # If it has children, recursively search each child
            for child in root:
                if child.tag == 'body' or child.tag == 'worldbody':
                    #print(child.tag)
                    find_leaf_nodes(child, leaves)
                pass
    except:
        pass




#Access or process your leaf nodes
# for leaf in leaf_nodes:
#     print(leaf.attrib)

def mutate(sur_dict, xml):
    tree = ET.parse(xml)
    root = tree.getroot()[1]
    leaf_nodes = []
    find_leaf_nodes(root, leaf_nodes)

    #*** ADD one more module in the end of XML file** ---------------------------
    for children in root.iter('body'):
        #print(children.attrib)
        name = children.get('name')
    modular.build_unit(children, f"module__{int(name[-12])+1}", "0 0 0")
    if f"{xml[:-4]}0.xml" in sur_dict:
        tree.write(f"{xml[:-4]}-a0.xml")
        return f"{xml[:-4]}-a0.xml"
    else:
        tree.write(f"{xml[:-4]}0.xml")
        return f"{xml[:-4]}0.xml"

def melt_dictionaries(d1, d2):
    # Create a new dictionary to store the result
    merged_dict = d1.copy()  # Start with a copy of the first dictionary
    
    for key, value in d2.items():
        # If the key exists in the first dictionary, modify the key from the second dictionary
        if key in merged_dict:
            new_key = f"{key[:-4]}-a0.xml"  # Add a zero to the end of the key
            merged_dict[new_key] = value
        else:
            # If the key does not exist, add it directly
            merged_dict[key] = value
            
    return merged_dict



#------ test ------------------------
# xml = "actuator0.xml"
# motion.fitness_cal(xml,1)
# mutate(xml)
# motion.fitness_cal(xml,1)

