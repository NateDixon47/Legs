import mujoco
import os
urdf_dir = "/home/nate/Legs-main/Legs-main"
m = mujoco.MjModel.from_xml_path(os.path.join(urdf_dir, 'legs_feet.urdf'))
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bipedal.xml")
mujoco.mj_saveLastXML(out, m)
print("wrote", out)