import numpy as np
import open3d as o3d

def convert_to_mesh_bpa(pcd):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    output = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))
    return output

def convert_to_mesh_poisson(pcd):
    output = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
    return output

def clean_mesh(mesh, decimated_size=int(1e5)):
    dec_mesh = mesh.simplify_quadric_decimation(decimated_size)
    dec_mesh.remove_degenerate_triangles()
    dec_mesh.remove_duplicated_triangles()
    dec_mesh.remove_duplicated_vertices()
    dec_mesh.remove_non_manifold_edges()
    return dec_mesh

if __name__ == '__main__':

    """
    note: convert_to_mesh_bpa > convert_to_mesh_poisson, in my experience
    """

    # %% load bunny and estimate normals
    pcd = o3d.io.read_point_cloud("data/bunny.pcd")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # %% convert using our functions
    bpa = convert_to_mesh_bpa(pcd)
    poisson = convert_to_mesh_poisson(pcd)

    # %% decimate, remove degenerates and duplicates
    cleaned_bpa = clean_mesh(bpa)
    cleaned_poisson = clean_mesh(poisson)

    # %% visualize in turn
    print('visualizing ball pivoting algorithm result')
    o3d.visualization.draw_geometries([pcd, cleaned_bpa])
    print('visualizing poisson result')
    o3d.visualization.draw_geometries([pcd, poisson])
