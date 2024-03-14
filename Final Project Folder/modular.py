import xml.etree.ElementTree as ET
import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
# tag means <mujoco>,<body>
# attrib means the gravity="0 0 -9.81" , the attribute in the <>
# text can also be stored between angle brakets <>

## unit define 
def build_unit(parent, name, pos):
    top = ET.SubElement(parent, "body", name = f"{name}module-top", pos = pos)
    free_plate = ET.SubElement(top, "joint", type = "hinge", axis = "1 0 0", range = "-45 45")
    geom_free_plate = ET.SubElement(top, "geom", type = "box",name = f"{name}geom_free_plate", size=".1 .5 .5", rgba ="0 0 0 1" ,mass="100")

    slide_box_body = ET.SubElement(top, "body", name = f"{name}slide_box", pos = "1 0.3 0.3")
    slide_box_joint = ET.SubElement(slide_box_body, "joint", name = f"{name}slide_box_joint", type = "slide", axis = "1 0 0",range="-0.5 0.5")
    geom_slide_box = ET.SubElement(slide_box_body, "geom", type = "cylinder",name = f"{name}slide_box_body", size="0.2 0.5", euler="0 90 0", rgba ="1 1 1 1" ,mass="400")

    hingex_box_body= ET.SubElement(slide_box_body, "body", name = f"{name}hingex_box_body", pos = "1 0 0")
    hingex_box_joint = ET.SubElement(hingex_box_body, "joint",name = f"{name}hingex_box_joint", type = "hinge" ,axis = "1 0 0",range="-45 45")
    geom_hingex_box =ET.SubElement(hingex_box_body, "geom", type = "cylinder", name = f"{name}geom_hingex_box", size ="0.2 0.5",euler=" 0 90 0", rgba = "1 1 1 1" ,mass="400")

    hingey_box_body = ET.SubElement(hingex_box_body, "body", name = f"{name}hingey_box_body", pos = "1 0 0")
    hingey_box_joint = ET.SubElement(hingey_box_body, "joint", name = f"{name}hingey_box_joint", type = "hinge", axis = "0 1 0",range="-45 45")
    geom_hingey_box = ET.SubElement(hingey_box_body, "geom" , type = "cylinder",name = f"{name}geom_hingey_box" ,size ="0.2 0.5",euler="0 90 0", rgba = "1 1 1 1" ,mass="400")

    hingez_box_body = ET.SubElement(hingey_box_body, "body", name = f"{name}hingez_box_body", pos = "1 0 0")
    hingez_box_joint = ET.SubElement(hingez_box_body, "joint", name = f"{name}hingez_box_joint", type = "hinge", axis = "0 0 1",range="-45 45")
    geom_hingez_box = ET.SubElement(hingez_box_body, "geom" , type = "cylinder", name = f"{name}geom_hingez_box" ,size ="0.2 0.5", euler="0 90 0",rgba = "1 1 1 1" ,mass="400")

    slide_plate = ET.SubElement(hingez_box_body, "body", name= f"{name}slide_plate", pos = "1 -0.3 -0.3")
    slide_plate_joint = ET.SubElement(slide_plate, "joint", name = f"{name}slide_plate_joint", type = "slide" , range="-0.5 0.5")
    geom_slide_plate = ET.SubElement(slide_plate, "geom", type = "box", name = f"{name}slide_plate", size ="0.1 .5 .5", rgba = "0 0 0 1",mass="400")

    return slide_plate