import numpy as np

def get_matrix(transform, file_path = None):

    # Creates matrix from carla transform.  
    # position_world = matritx * position_lidar

    rotation = transform.rotation
    location = transform.location
    # c_y = np.cos(np.radians(rotation.yaw))
    # s_y = np.sin(np.radians(rotation.yaw))
    # c_r = np.cos(np.radians(rotation.roll))
    # s_r = np.sin(np.radians(rotation.roll))
    
    # to nagtive in right-handed
    c_y = np.cos(np.radians(-rotation.yaw))
    s_y = np.sin(np.radians(-rotation.yaw))
    c_r = np.cos(np.radians(-rotation.roll))
    s_r = np.sin(np.radians(-rotation.roll))

    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))
    
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = location.x
    # matrix[1, 3] = location.y
    matrix[1, 3] = -location.y  # to nagtive in right-handed
    matrix[2, 3] = location.z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r

    if file_path is not None:
        np.savetxt(file_path, matrix, delimiter=' ', )

    return matrix
    
    

def test(obj_transform_in_world, lidar_transform_in_world, sensor_location):

    lc = obj_transform_in_world.location
    loc_w = np.matrix([lc.x, -lc.y, lc.z, 1])  # to nagtive in right-handed
    print('\nloc in world(right-handed):\n', loc_w)

    s_l = sensor_location
    loc_l = np.matrix([-s_l.x, s_l.y, -s_l.z, 1])
    # loc_l = np.matrix([0.5, -0.5, -2.095, 1])
    print('loc in lidar(right-handed):\n', loc_l)

    matrix = get_matrix(lidar_transform_in_world)
    print('lidar to world matrix:\n', matrix)
    
    pos_w = np.dot(matrix, loc_l.T)
    print('loc in world(right-handed)counted by matrix:\n', pos_w)

    pos_l = np.dot(matrix.I, loc_w.T)
    print('loc in lidar(right-handed)counted by matrix:\n', pos_l)