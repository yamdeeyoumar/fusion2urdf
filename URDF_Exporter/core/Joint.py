# Modified `make_joints_dict` from Joint.py with axis fix for nested links
import adsk, re, math
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

    def make_joint_xml(self):
        joint = Element('joint')
        joint.attrib = {'name': self.name, 'type': self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz': ' '.join([str(_) for _ in self.xyz]), 'rpy': '0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link': self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link': self.child}
        if self.type in ['revolute', 'continuous', 'prismatic']:
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz': ' '.join([str(_) for _ in self.axis])}
        if self.type in ['revolute', 'prismatic']:
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}

        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        tran = Element('transmission')
        tran.attrib = {'name': self.name + '_tran'}

        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'

        joint = SubElement(tran, 'joint')
        joint.attrib = {'name': self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'

        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name': self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'

        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    joint_type_list = ['fixed', 'revolute', 'prismatic', 'Cylinderical',
                       'PinSlot', 'Planner', 'Ball']

    joints_dict = {}

    def rotmat_to_rpy(R):
        sy = math.sqrt(R[0]**2 + R[3]**2)
        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(R[7], R[8])
            pitch = math.atan2(-R[6], sy)
            yaw = math.atan2(R[3], R[0])
        else:
            roll = math.atan2(-R[5], R[4])
            pitch = math.atan2(-R[6], sy)
            yaw = 0

        return [round(roll, 6), round(pitch, 6), round(yaw, 6)]

    def rotate_vector(M, v):
        return [
            M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
            M[4] * v[0] + M[5] * v[1] + M[6] * v[2],
            M[8] * v[0] + M[9] * v[1] + M[10] * v[2]
        ]
    
    def rotate_vector_inv(M, v):
    # world→local (Rᵀ·v)
        return [
            M[0] * v[0] + M[4] * v[1] + M[8]  * v[2],
            M[1] * v[0] + M[5] * v[1] + M[9]  * v[2],
            M[2] * v[0] + M[6] * v[1] + M[10] * v[2]
        ]

    def trans(M, a):
        ex = [M[0], M[4], M[8]]
        ey = [M[1], M[5], M[9]]
        ez = [M[2], M[6], M[10]]
        oo = [M[3], M[7], M[11]]
        return [sum(a[i] * ex[i] for i in range(3)) + oo[0],
                sum(a[i] * ey[i] for i in range(3)) + oo[1],
                sum(a[i] * ez[i] for i in range(3)) + oo[2]]

    def allclose(v1, v2, tol=1e-6):
        return max([abs(a - b) for a, b in zip(v1, v2)]) < tol

    for joint in root.joints:
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0

        try:
            M_parent = joint.occurrenceTwo.transform.asArray()
            M_child = joint.occurrenceOne.transform.asArray()
            def relative_rotation_rpy(M1, M2):
                # Extract 3x3 rotation matrices
                R1 = [M1[0], M1[1], M1[2], M1[4], M1[5], M1[6], M1[8], M1[9], M1[10]]
                R2 = [M2[0], M2[1], M2[2], M2[4], M2[5], M2[6], M2[8], M2[9], M2[10]]

                # Transpose R1 (i.e., invert rotation)
                R1_T = [
                    R1[0], R1[3], R1[6],
                    R1[1], R1[4], R1[7],
                    R1[2], R1[5], R1[8]
                ]

                # Multiply R1_T * R2
                R_rel = [
                    sum(R1_T[i*3 + k] * R2[k*3 + j] for k in range(3))
                    for i in range(3) for j in range(3)
                ]
                return rotmat_to_rpy(R_rel)

            # Compute relative RPY and store in joint_dict
            joint_dict['rpy'] = relative_rotation_rpy(M_parent, M_child)
            if joint_type == 'revolute':
                raw_global  = joint.jointMotion.rotationAxisVector.asArray()
                # convert world→parent frame
                local_axis = rotate_vector_inv(M_child, raw_global)
                norm = (local_axis[0]**2 + local_axis[1]**2 + local_axis[2]**2)**0.5
                joint_dict['axis'] = [round(i/norm, 6) for i in local_axis]
                if joint.jointMotion.rotationLimits.isMaximumValueEnabled and joint.jointMotion.rotationLimits.isMinimumValueEnabled:
                    joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                    joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
                else:
                    joint_dict['type'] = 'continuous'
            elif joint_type == 'prismatic':
                raw_axis = joint.jointMotion.slideDirectionVector.asArray()
                global_axis = rotate_vector(M_parent, raw_axis)
                norm = sum(i ** 2 for i in global_axis) ** 0.5
                joint_dict['axis'] = [round(i / norm, 6) for i in global_axis]
                if joint.jointMotion.slideLimits.isMaximumValueEnabled and joint.jointMotion.slideLimits.isMinimumValueEnabled:
                    joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue / 100, 6)
                    joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue / 100, 6)
        except:
            msg = f"{joint.name} axis transformation failed."
            break

        joint_dict['parent'] = 'base_link' if joint.occurrenceTwo.component.name == 'base_link' else re.sub('[ :()]', '_', joint.occurrenceTwo.name)
        joint_dict['child'] = re.sub('[ :()]', '_', joint.occurrenceOne.name)

        try:
            xyz_of_joint = trans(M_parent, joint.geometryOrOriginTwo.origin.asArray())
            joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint]
        except:
            msg = joint.name + " doesn't have joint origin."
            break

        joints_dict[joint.name] = joint_dict

    return joints_dict, msg
