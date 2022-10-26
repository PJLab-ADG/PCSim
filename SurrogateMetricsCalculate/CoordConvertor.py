import math
from pickle import TRUE
from pickletools import uint4
from tkinter.messagebox import NO
import numpy as np
import shutil

########################################################################################################
#----CarlaLidarDataCoordinateConvertor
########################################################################################################
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

def pcd_coordinate_convertor(file, trans_mat):
    with open(file, 'r') as r:
        with open(file+'.new', 'w') as w:
            for l in r.readlines():
                # print(l.split(' ')[0])
                if is_number(l.split(' ')[0]):
                    #print(l)

                    # loc = np.matrix([float(l[0]), float(l[1]), float(l[2]), 1.0])
                    x = float(l.split(' ')[0])
                    y = float(l.split(' ')[1])
                    z = float(l.split(' ')[2])
                    loc = np.matrix([x, y, z, 1.0])

                    loc_target = np.dot(trans_mat, loc.T)
                    loc_target = loc_target.tolist()
                    # print(loc_target)

                    l_new_list = [0, 0, 0, 0, 0, 0]

                    l_new_list[0] = str(loc_target[0][0])
                    l_new_list[1] = str(loc_target[1][0])
                    l_new_list[2] = str(loc_target[2][0])
                    l_new_list[3] = l.split(' ')[3]
                    l_new_list[4] = l.split(' ')[4]
                    l_new_list[5] = l.split(' ')[5]
                    l_new = ' '.join(l_new_list)
                    w.write(l_new)

                else:
                    w.write(l)
                # if 'nan' not in l:
                #     w.write(l)
    shutil.move(file+'.new', file)
