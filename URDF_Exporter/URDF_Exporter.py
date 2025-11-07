# Author-syuntoku14
# Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the coordinates are wrong.
"""

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 
        components = design.allComponents

        # Generate joints_dict. All joints are related to root. 
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0   

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0

        links_xyz_dict = Link.make_links_xyz_dict(root)

        # --------------------
        # set save directory
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        save_dir = save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass

        # Write debug info to file after save_dir is ready
        debug_file_path = os.path.join(save_dir, 'fusion2urdf_debug_output.txt')
        with open(debug_file_path, 'w') as debug_file:
            debug_file.write("=== JOINTS DICT ===\n")
            for key, val in joints_dict.items():
                debug_file.write(f"{key}: {val}\n")
            debug_file.write("\n=== INERTIAL DICT ===\n")
            for key, val in inertial_dict.items():
                debug_file.write(f"{key}: {val}\n")
            debug_file.write("\n=== LINKS XYZ DICT ===\n")
            for key, val in links_xyz_dict.items():
                debug_file.write(f"{key}: {val}\n")
            debug_file.write("\n=== JOINT FRAME AXES, ORIGINS, AND AXIS VECTORS ===\n")
            for joint in root.joints:
                try:
                    name = joint.name
                    M_parent = joint.occurrenceTwo.transform.asArray()
                    M_child = joint.occurrenceOne.transform.asArray()

                    # Rotation matrix axes of parent link (occurrenceTwo)
                    x_axis = [M_parent[0], M_parent[4], M_parent[8]]
                    y_axis = [M_parent[1], M_parent[5], M_parent[9]]
                    z_axis = [M_parent[2], M_parent[6], M_parent[10]]

                    def normalize(v):
                        norm = sum(i**2 for i in v) ** 0.5
                        return [round(i / norm, 6) for i in v]

                    # Origin of joint in world frame
                    origin = joint.geometryOrOriginTwo.origin.asArray()
                    origin_m = [round(o / 100.0, 6) for o in origin]

                    # Axis in world frame
                    axis_world = joint.jointMotion.rotationAxisVector.asArray()
                    axis_world_norm = normalize(axis_world)

                    # Axis in child frame
                    def rotate_vector_inv(M, v):
                        return [
                            M[0]*v[0] + M[4]*v[1] + M[8]*v[2],
                            M[1]*v[0] + M[5]*v[1] + M[9]*v[2],
                            M[2]*v[0] + M[6]*v[1] + M[10]*v[2]
                        ]
                    axis_child = normalize(rotate_vector_inv(M_child, axis_world))

                    debug_file.write(f"{name}:\n")
                    debug_file.write(f"  Origin (m): {origin_m}\n")
                    debug_file.write(f"  Axis (world frame): {axis_world_norm}\n")
                    debug_file.write(f"  Axis (child frame): {axis_child}\n")
                    debug_file.write(f"  Parent X-axis: {normalize(x_axis)}\n")
                    debug_file.write(f"  Parent Y-axis: {normalize(y_axis)}\n")
                    debug_file.write(f"  Parent Z-axis: {normalize(z_axis)}\n")
                    # World coordinates of joint origin
                    origin_world = joint.geometryOrOriginTwo.origin.asArray()
                    origin_world_m = [round(o / 100.0, 6) for o in origin_world]
                    debug_file.write(f"  Joint origin (world): {origin_world_m}\n")

                    # Center of mass of parent and child
                    parent_prop = joint.occurrenceTwo.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                    child_prop = joint.occurrenceOne.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)

                    parent_com = [round(c / 100.0, 6) for c in parent_prop.centerOfMass.asArray()]
                    child_com = [round(c / 100.0, 6) for c in child_prop.centerOfMass.asArray()]
                    debug_file.write(f"  Parent CoM (world): {parent_com}\n")
                    debug_file.write(f"  Child CoM (world): {child_com}\n")

                    debug_file.write("\n")
                    rpy = joints_dict[joint.name].get('rpy', [0.0, 0.0, 0.0])
                    debug_file.write(f"  Relative RPY (parent -> child): {rpy}\n")
                except Exception as e:
                    debug_file.write(f"{joint.name}: Failed to extract data: {str(e)}\n")


        try:
            os.startfile(debug_file_path)
        except:
            ui.messageBox(f"Saved debug output to: {debug_file_path}")

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'

        # --------------------
        # Generate URDF files
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir)
        Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
        Write.write_yaml(package_name, robot_name, save_dir, joints_dict)

        # Copy over package files        
        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # Generate STL files        
        name_map = utils.copy_occs(root)
        utils.export_stl(design, save_dir, root, name_map)  

        # Final message
        ui.messageBox(msg, title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
