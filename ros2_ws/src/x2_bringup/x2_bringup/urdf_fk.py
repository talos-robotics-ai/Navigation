"""Tiny pure-numpy URDF forward kinematics (revolute/continuous/prismatic/fixed; no deps).

    urdf = Urdf.load(path)
    T = urdf.fk('base_link', 'rgbd_head_front', {'waist_yaw_joint': 0.1, ...})   # T_base_cam
Joints missing from `q` are at 0. Only the chains between the requested links are evaluated.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rot_axis(axis, ang):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    a = a / n if n else np.array([0.0, 0.0, 1.0])
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Urdf:
    def __init__(self, joints):
        # child link -> (parent link, joint name, type, T_parent_joint, axis)
        self.joints = joints

    @classmethod
    def load(cls, path):
        joints = {}
        for j in ET.parse(path).getroot().iter('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = _rpy(*rpy), xyz
            ax = j.find('axis')
            axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [0.0, 0.0, 1.0]
            joints[j.find('child').get('link')] = (j.find('parent').get('link'), j.get('name'),
                                                   j.get('type'), T, axis)
        return cls(joints)

    def chain_joints(self, link):
        """Names of the non-fixed joints between `link` and the root."""
        out = []
        while link in self.joints:
            parent, name, typ, _, _ = self.joints[link]
            if typ != 'fixed':
                out.append(name)
            link = parent
        return out

    def _to_root(self, link, q):
        """(root link name, T_root_link)."""
        T = np.eye(4)
        while link in self.joints:
            parent, name, typ, Tpj, axis = self.joints[link]
            Tj = Tpj.copy()
            v = float(q.get(name, 0.0))
            if typ in ('revolute', 'continuous'):
                Tj[:3, :3] = Tpj[:3, :3] @ _rot_axis(axis, v)
            elif typ == 'prismatic':
                Tj[:3, 3] = Tpj[:3, 3] + Tpj[:3, :3] @ (np.asarray(axis, float) * v)
            T = Tj @ T
            link = parent
        return link, T

    def fk(self, from_link, to_link, q=None):
        """T_from_to."""
        q = q or {}
        r1, T1 = self._to_root(from_link, q)
        r2, T2 = self._to_root(to_link, q)
        if r1 != r2:
            raise ValueError(f'{from_link} and {to_link} are not in one tree')
        Ti = np.eye(4)
        Ti[:3, :3] = T1[:3, :3].T
        Ti[:3, 3] = -T1[:3, :3].T @ T1[:3, 3]
        return Ti @ T2
