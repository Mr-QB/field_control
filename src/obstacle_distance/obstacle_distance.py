import os
import math
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import shape_msgs.msg
import trimesh
from .urdf_kinematics import URDFKinematics


def resolve_package_url(url):
    """Resolve package://pkg_name/path/to/file to an absolute filesystem path."""
    if url and url.startswith('package://'):
        parts = url[len('package://'):].split('/', 1)
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory(parts[0]), parts[1] if len(parts) > 1 else '')
        except Exception:
            pass
    return url or ""

 
def pose_to_matrix(pose):
    """Convert geometry_msgs/Pose to a 4x4 homogeneous transformation matrix."""
    T = np.identity(4)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    norm = np.linalg.norm(q)
    if norm > 1e-6:
        T[:3, :3] = R.from_quat([x / norm for x in q]).as_matrix()
    return T


def mesh_to_box_distance(verts_world, box_T, dims):
    """Calculate minimum distance from CAD mesh vertices (in world frame) to a 3D solid box obstacle."""
    inv_T = np.linalg.inv(box_T)
    v_loc = (inv_T @ np.hstack([verts_world, np.ones((len(verts_world), 1))]).T).T[:, :3]
    half_extents = np.array(dims) / 2.0

    dists = np.linalg.norm(np.maximum(np.abs(v_loc) - half_extents, 0.0), axis=1)
    min_idx = np.argmin(dists)

    cpt_box_loc = np.sign(v_loc[min_idx]) * np.minimum(np.abs(v_loc[min_idx]), half_extents)
    cpt_box_world = (box_T @ np.append(cpt_box_loc, 1.0))[:3]
    return float(dists[min_idx]), verts_world[min_idx], cpt_box_world


def mesh_to_sphere_distance(verts_world, center, radius):
    """Calculate minimum distance from CAD mesh vertices (in world frame) to a 3D solid sphere obstacle."""
    vecs = verts_world - center
    dists_to_center = np.linalg.norm(vecs, axis=1)
    surf_dists = np.maximum(0.0, dists_to_center - radius)

    min_idx = np.argmin(surf_dists)
    d_c = dists_to_center[min_idx]
    cpt_sphere = center if d_c == 0.0 else center + (vecs[min_idx] / d_c) * radius
    return float(surf_dists[min_idx]), verts_world[min_idx], cpt_sphere


def mesh_to_cylinder_distance(verts_world, cyl_T, height, radius):
    """Calculate minimum distance from CAD mesh vertices (in world frame) to a 3D solid cylinder obstacle."""
    inv_T = np.linalg.inv(cyl_T)
    v_loc = (inv_T @ np.hstack([verts_world, np.ones((len(verts_world), 1))]).T).T[:, :3]
    half_h = height / 2.0

    r_xy = np.linalg.norm(v_loc[:, :2], axis=1)
    dists = np.sqrt(np.maximum(0.0, r_xy - radius)**2 + np.maximum(0.0, np.abs(v_loc[:, 2]) - half_h)**2)
    min_idx = np.argmin(dists)

    p_loc, r_p = v_loc[min_idx], r_xy[min_idx]
    xy = (p_loc[:2] / r_p) * min(r_p, radius) if r_p > 0 else np.zeros(2)
    cpt_cyl_loc = np.array([xy[0], xy[1], max(-half_h, min(half_h, p_loc[2]))])
    cpt_cyl_world = (cyl_T @ np.append(cpt_cyl_loc, 1.0))[:3]
    return float(dists[min_idx]), verts_world[min_idx], cpt_cyl_world


class SolidObstacleDistanceCalculator:
    """
    Generic, Robot-Independent Real-Time Distance Calculator.
    Automatically parses URDF collision meshes & YAML configuration files.
    """

    def __init__(self, config_path=None, config_dict=None, urdf_path=None, mesh_dir=None, link_mesh_map=None, urdf_content=None):
        cfg = {}
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                cfg = data.get('obstacle_distance_node', {}).get('ros__parameters', {}) if 'obstacle_distance_node' in data else data

        if config_dict:
            cfg.update(config_dict)

        urdf_path = urdf_path or cfg.get('urdf_path', '')
        mesh_dir = mesh_dir or cfg.get('mesh_dir', '')
        link_mesh_map = link_mesh_map or cfg.get('link_mesh_map', {})

        self.kin = URDFKinematics(urdf_path=urdf_path if urdf_path else None, urdf_content=urdf_content)
        self.link_meshes = {}

        # 1. Parse collision meshes directly from URDF link definitions
        self._load_meshes_from_urdf()

        # 2. Supplement meshes from explicit mesh_dir & link_mesh_map if provided
        if mesh_dir and link_mesh_map:
            for link_name, filename in link_mesh_map.items():
                stl_path = os.path.join(mesh_dir, filename)
                if os.path.exists(stl_path):
                    try:
                        self.link_meshes[link_name] = trimesh.load(stl_path).vertices
                    except Exception:
                        pass

    def _load_meshes_from_urdf(self):
        """Parse collision meshes directly from robot URDF link definitions."""
        for link in getattr(self.kin.robot, 'links', []):
            for col in getattr(link, 'collisions', []) or []:
                fn = getattr(col.geometry, 'filename', None)
                if fn:
                    resolved_path = resolve_package_url(fn)
                    if os.path.exists(resolved_path):
                        try:
                            self.link_meshes[link.name] = trimesh.load(resolved_path).vertices
                            break
                        except Exception:
                            pass

    def compute_distance_to_primitive(self, verts_world, primitive, primitive_pose_mat):
        pt_type, dims = primitive.type, list(primitive.dimensions)
        if pt_type == shape_msgs.msg.SolidPrimitive.BOX:
            return mesh_to_box_distance(verts_world, primitive_pose_mat, dims)
        elif pt_type == shape_msgs.msg.SolidPrimitive.SPHERE:
            return mesh_to_sphere_distance(verts_world, primitive_pose_mat[:3, 3], dims[shape_msgs.msg.SolidPrimitive.SPHERE_RADIUS])
        elif pt_type == shape_msgs.msg.SolidPrimitive.CYLINDER:
            return mesh_to_cylinder_distance(verts_world, primitive_pose_mat, dims[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT], dims[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS])
        return mesh_to_sphere_distance(verts_world, primitive_pose_mat[:3, 3], 0.05)

    def compute_per_link_distances(self, q, obstacles, base_link='base_link', tip_link='tool0'):
        path_links, _ = self.kin.get_chain(base_link=base_link, tip_link=tip_link)
        transforms = self.kin.forward_kinematics(q, base_link=base_link, tip_link=tip_link)

        per_link_results = {}
        overall = {'min_dist': float('inf'), 'link': None, 'obs': None, 'cpt_link': None, 'cpt_obs': None}

        for link_name in path_links:
            if link_name in transforms:
                T_link = transforms[link_name]
                local_verts = self.link_meshes.get(link_name)
                verts_world = np.array([T_link[:3, 3]]) if local_verts is None else (T_link @ np.hstack([local_verts, np.ones((len(local_verts), 1))]).T).T[:, :3]

                link_min_dist, closest_obs_id, cpt_l, cpt_o = float('inf'), None, None, None

                for obs in obstacles:
                    obs_id = obs.get('id', 'unknown')
                    for prim, p_pose in zip(obs.get('primitives', []), obs.get('primitive_poses', [])):
                        p_mat = p_pose if isinstance(p_pose, np.ndarray) else pose_to_matrix(p_pose)
                        dist, pt_mesh, pt_obs = self.compute_distance_to_primitive(verts_world, prim, p_mat)

                        if dist < link_min_dist:
                            link_min_dist, closest_obs_id = dist, obs_id
                            cpt_l, cpt_o = pt_mesh.tolist(), pt_obs.tolist()

                per_link_results[link_name] = {
                    'link_name': link_name,
                    'distance': link_min_dist if link_min_dist != float('inf') else -1.0,
                    'closest_obstacle_id': closest_obs_id,
                    'closest_point_on_link': cpt_l,
                    'closest_point_on_obstacle': cpt_o,
                }

                if link_min_dist < overall['min_dist']:
                    overall.update({'min_dist': link_min_dist, 'link': link_name, 'obs': closest_obs_id, 'cpt_link': cpt_l, 'cpt_obs': cpt_o})

        return {
            'per_link': per_link_results,
            'overall_min_distance': overall['min_dist'] if overall['min_dist'] != float('inf') else -1.0,
            'overall_closest_link': overall['link'],
            'overall_closest_obstacle': overall['obs'],
            'overall_closest_point_link': overall['cpt_link'],
            'overall_closest_point_obstacle': overall['cpt_obs'],
        }
