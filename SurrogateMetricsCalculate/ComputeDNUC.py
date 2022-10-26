import shutil
import random
import math
import os

def computeNUC(file, d, disk_radius, p, target_field):
    '''
    file -- semantic ply file path
    d -- the number of disk
    disk_radius -- the radius of disk
    p -- disk area percentage
    '''
    point_list = semantic_filter(file, target_field)
    total_point = len(point_list) # total number of point
    random_d_index = random.sample(range(total_point), d) # choose d indexes from total_point randomly, d must be smaller than total_point
    total_nums_in_disk = 0

    tmp_val = [] # n_i / (N * p)
    for index in random_d_index:
        nums_in_index = nums_in_disk(point_list, index, disk_radius) # compute the point number in index_th disk
        total_nums_in_disk = total_nums_in_disk + nums_in_index
        tmp_val.append(nums_in_index / (total_point *p))

    avg = total_nums_in_disk / (d * total_point * p)

    nuc = 0
    for val in tmp_val:
        nuc = nuc + math.pow(val - avg, 2)
    nuc = math.sqrt(nuc / d)

    return total_point, nuc

def nums_in_disk(point_list, point_index, disk_radius):
    disk_center_point = point_list[point_index]
    d_x = disk_center_point[0]
    d_y = disk_center_point[1]
    # d_z = disk_center_point[2]
    point_nums = 0
    for point in point_list:
        p_x = point[0]
        p_y = point[1]
        # p_z = point[2]
        dist = math.sqrt(math.pow(d_x - p_x, 2) + math.pow(d_y - p_y, 2))
        if dist <= disk_radius:
            point_nums = point_nums + 1
    return point_nums


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        pass
    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass
    return False

def semantic_filter(file, target_field):
    target_tag_1 = 7 # 7 means Road
    target_tag_2 = 8 # 8 means SideWalk
    filtered_list = []
    with open(file, 'r') as r:
        with open(file+'.new', 'w') as w:
            for l in r.readlines():
                # print(l.split(' ')[0])
                if is_number(l.split(' ')[0]):
                    if int(l.split(' ')[5]) != target_tag_1 and int(l.split(' ')[5]) != target_tag_2:
                        continue
                    l_x = float(l.split(' ')[0])
                    l_y = float(l.split(' ')[1])
                    if (l_x < target_field[0]) or (l_x > target_field[1]) or (l_y < target_field[2]) or (l_y > target_field[3]):
                        continue
                    l_new_list = [0, 0, 0, 0, 0, 0]
                    l_new_list[0] = l.split(' ')[0]
                    l_new_list[1] = l.split(' ')[1]
                    l_new_list[2] = l.split(' ')[2]
                    l_new_list[3] = l.split(' ')[3]
                    l_new_list[4] = l.split(' ')[4]
                    l_new_list[5] = l.split(' ')[5]

                    pos_list = [float(l.split(' ')[0]), float(l.split(' ')[1]), float(l.split(' ')[2])] # save (x, y, z)
                    filtered_list.append(pos_list)

                    l_new = ' '.join(l_new_list)
                    w.write(l_new)

                else:
                    w.write(l)
                
    with open(file+'.new', 'r') as rd:
        with open(file + '.filtered', 'w') as wt:
            for i, line in enumerate(rd.readlines()):
                if i == 2:
                    line_new = []
                    line_new.append(line.split(' ')[0])
                    line_new.append(line.split(' ')[1])
                    line_new.append(str(len(filtered_list)))
                    write_str = ' '.join(line_new) + '\n'
                    wt.write(write_str)
                else:
                    wt.write(line)
    os.remove(file+'.new')
    new_name = file[:-4] + '_filtered.ply'  
    shutil.move(file + '.filtered', new_name)
    return filtered_list



# ---------test------------
index = 1
# file = './7_placement.ply'
d = 6000
disk_radius = 0.7
target_field = [92, 112, 80, 110]
area = (target_field[1] - target_field[0]) * (target_field[3] - target_field[2])
p = math.pi * (disk_radius**2) / area



for index in range(6, 9):
    file = './' + str(index) + '.ply'
    total_nums, NUC = computeNUC(file, d, disk_radius, p, target_field)
    print("placement_"+str(index))
    # print ("total_nums:", total_nums, "\nNUC:", NUC)
    print ("density:", total_nums/area, "\nNUC:", NUC, '\n')
