from urdfpy import URDF
block = URDF.load('block.urdf') 

visual = block.visual_trimesh_fk()
# visual = block.link_fk()
print(type(list(visual.values())[0]))
print(list(visual.values())[0])


for link in block.links:
    print(link.name)
    
    
# for link in block.links:
#     print(link.visuals.__dict__)
    

# with open("box.urdf", "w") as file_descriptor:
#     yaml.dump(yaml_dict, file_descriptor)

kappa = """<?xml version="1.0"?>
<robot name="block_2">
  <link name="block_2_base_link">
    <contact>
      <lateral_friction value="1.0"/>
      <spinning_friction value=".001"/>
    </contact>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.10 1.0 2.0"/>
      </geometry>
      <material name="blockmat">
        <color rgba="0.1 0.7 0.1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.10 0.018 0.018"/>
      </geometry>
    </collision>
  </link>
</robot>"""

f = open("box.urdf", "w")
f.write(kappa)
f.close()