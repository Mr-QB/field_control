import math
import subprocess
import numpy as np
from urdf_parser_py.urdf import URDF


def rpy_to_matrix(rpy):
    """Convert RPY angles (extrinsic XYZ) to a 4x4 homogeneous rotation matrix."""
    if not rpy:
        return np.identity(4)
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0, 0], [0, cr, -sr, 0], [0, sr, cr, 0], [0, 0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp, 0], [0, 1, 0, 0], [-sp, 0, cp, 0], [0, 0, 0, 1]], dtype=float)
    Rz = np.array([[cy, -sy, 0, 0], [sy, cy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def translation_matrix(xyz):
    """Convert Translation [x, y, z] to a 4x4 homogeneous translation matrix."""
    T = np.identity(4)
    if xyz:
        T[:3, 3] = xyz
    return T


def axis_rotation_matrix(axis, theta):
    """Generate a 4x4 homogeneous rotation matrix around axis by theta (Rodrigues formula)."""
    axis = np.array(axis if axis else [1, 0, 0], dtype=float)
    norm = np.linalg.norm(axis)
    if norm > 0:
        axis /= norm
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return np.array([
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s, 0.0],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s, 0.0],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c,   0.0],
        [0.0,         0.0,         0.0,         1.0]
    ])


def point_to_segment_distance(pt, seg_a, seg_b):
    """Calculate the shortest distance from a 3D point to a 3D line segment."""
    pt, seg_a, seg_b = np.array(pt), np.array(seg_a), np.array(seg_b)
    ab = seg_b - seg_a
    ab_sq = np.dot(ab, ab)
    t = 0.0 if ab_sq == 0.0 else max(0.0, min(1.0, np.dot(pt - seg_a, ab) / ab_sq))
    closest_pt = seg_a + t * ab
    return float(np.linalg.norm(pt - closest_pt)), closest_pt


class URDFKinematics:
    """Computes the forward kinematics of ANY robot using its URDF XML or file path."""

    def __init__(self, urdf_path=None, urdf_content=None, xacro_args=None):
        if not urdf_content:
            if not urdf_path:
                raise ValueError("Neither 'urdf_content' nor 'urdf_path' was provided to URDFKinematics.")

            if urdf_path.endswith('.xacro'):
                cmd = ['xacro', urdf_path]
                if xacro_args:
                    if isinstance(xacro_args, list):
                        cmd.extend(xacro_args)
                    elif isinstance(xacro_args, dict):
                        for k, v in xacro_args.items():
                            cmd.append(f'{k}:={v}')

                try:
                    urdf_content = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode('utf-8')
                except Exception:
                    # Fallback retry with default UR xacro parameters if required
                    cmd_fallback = ['xacro', urdf_path, 'name:=ur', 'ur_type:=ur3e']
                    urdf_content = subprocess.check_output(cmd_fallback, stderr=subprocess.DEVNULL).decode('utf-8')
            else:
                with open(urdf_path, 'r') as f:
                    urdf_content = f.read()

        self.robot = URDF.from_xml_string(urdf_content)
        self.link_parent_joint = {joint.child: joint for joint in self.robot.joints}

    def get_chain(self, base_link='base_link', tip_link='tool0'):
        """Trace active kinematic chain of joints and links from base to tip."""
        path_links, path_joints = [], []
        curr = tip_link
        while curr != base_link and curr in self.link_parent_joint:
            joint = self.link_parent_joint[curr]
            path_joints.append(joint)
            path_links.append(curr)
            curr = joint.parent
        path_links.append(base_link)
        return path_links[::-1], path_joints[::-1]

    def forward_kinematics(self, q, base_link='base_link', tip_link='tool0'):
        """Compute 4x4 homogeneous forward kinematics frames for all links."""
        path_links, path_joints = self.get_chain(base_link, tip_link)
        transforms = {base_link: np.identity(4)}
        q_idx = 0

        for joint in path_joints:
            T_static = translation_matrix(joint.origin.xyz if joint.origin else None) @ \
                       rpy_to_matrix(joint.origin.rpy if joint.origin else None)

            if joint.type in ['revolute', 'continuous']:
                T_dynamic = axis_rotation_matrix(joint.axis, q[q_idx])
                q_idx += 1
            elif joint.type == 'prismatic':
                T_dynamic = translation_matrix(np.array(joint.axis if joint.axis else [1, 0, 0]) * q[q_idx])
                q_idx += 1
            else:
                T_dynamic = np.identity(4)

            transforms[joint.child] = transforms[joint.parent] @ (T_static @ T_dynamic)

        return transforms
