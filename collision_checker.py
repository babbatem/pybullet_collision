import os
import time
import numpy as np

import pybullet as p
import pybullet_data
import open3d as o3d

from utils import convert_to_mesh_bpa, clean_mesh

# %%
URDFPATH='data/gripper_urdf/gripper.urdf'
PLANE_POSE=[0, 0, -1]

class CollisionChecker(object):
    def __init__(self, vis=False):

        # %% setup pybullet
        self.vis=vis
        if vis:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setGravity(0,0,0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeUid=p.loadURDF("plane.urdf", PLANE_POSE)
        self.gripperUid=p.loadURDF(URDFPATH)

        self.dir = os.path.dirname(__file__)


    def check_poses(self, pos, ori, q, pcd):
        """
        pos: N x 3
        ori: N x 4 [x,y,z,w]
        q  : N x 12
        pcd: open3d pointcloud with normals, in world frame
        """

        # %% make a mesh
        mesh = convert_to_mesh_bpa(pcd)
        # mesh = clean_mesh(mesh, decimated_size=int(1e5))

        # %% write it
        fname_mesh = os.path.join(self.dir, "data/my_mesh.STL")
        o3d.io.write_triangle_mesh(fname_mesh, mesh)

        # %% load urdf
        fname_urdf = os.path.join(self.dir, "data/dummy.urdf")
        meshUid = p.loadURDF(fname_urdf,globalScaling=1)

        for i in range(len(pos)):

            # %% reset mesh for good measure
            p.resetBasePositionAndOrientation(meshUid,[0,0,0],[0,0,0,1])

            # %% set palm pose
            p.resetBasePositionAndOrientation(self.gripperUid,
                                              pos[i],
                                              ori[i])

            # %% set joint angles
            for j in range(p.getNumJoints(self.gripperUid)):
                p.resetJointState(self.gripperUid, j, q[i,j],0)

            # %% gotta step for p.direct...
            p.stepSimulation()

            # %% check collision
            contactPoints = p.getContactPoints(self.gripperUid,
                                               meshUid)

            if self.vis:
                if len(contactPoints) > 0:
                    print('Contacts: %i ' % len(contactPoints) )
                time.sleep(0.1)


if __name__ == '__main__':

    pcd = o3d.io.read_point_cloud("data/bunny.pcd")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    N = int(1e2)
    pos = np.random.rand(N*3).reshape(N,3)
    pos /= 10
    ori = np.random.rand(N*4).reshape(N,4)
    ori /= np.linalg.norm(ori)
    qs = np.random.rand(N*12).reshape(N,12)

    cc = CollisionChecker(vis=True)
    cc.check_poses(pos, ori, qs, pcd)
