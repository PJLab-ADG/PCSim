import json

with open('/home/PJLAB/caixinyu/carla/Unreal/CarlaUE4/Content/waymo_mesh_map/Config/waymo_mesh_map.Package.json',
          'r') as f:
    js = json.load(f)

sequence_all_name = []
with open('/home/PJLAB/caixinyu/Documents/Waymo-Sim/src/config/sequence_index.txt', 'r') as f:
    for line in f:
        sequence_all_name.append(line.split(' ')[0])
print(sequence_all_name)
with open('/home/PJLAB/caixinyu/Documents/Waymo-Sim/src/config/sequence_replay.txt', 'w') as f:
    for dic in js['props']:
        idx = dic['name'][3:10]
        start = 'segment-' + idx
        for item in sequence_all_name:
            if item.startswith(start):
                f.write(item + '\n')
