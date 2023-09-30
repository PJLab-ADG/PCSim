import glob
import os
import os.path as osp
import sys

import torch.utils.data as data

import inten
import otils as ot
import torchutils as tu

if __name__ == '__main__':
    config = ot.io.load_multi_yml(sys.argv[1])
    if 'seed' in config:
        tu.seed_all(config['seed'])
    config['store_dir'] = config['checkpoint']
    checkpoints = sorted([int(item[:-4]) for item in os.listdir(config['checkpoint']) if str(item).endswith('.tar')])
    config['checkpoint'] = osp.join(config['checkpoint'], str(checkpoints[-1]) + '.tar')
    runner = inten.data.EvalRunner(config)
    cp = ot.checkpoint.load_checkpoint(osp.join(config['base_dir'], config['store_dir'], config['checkpoint']), False)
    trn_dataset = data.DataLoader(inten.data.Dataset(config['train']), **config['train_loader'])
    val_dataset = data.DataLoader(inten.data.Dataset(config['val']), **config['val_loader'])
    runner(trn_dataset, osp.join(config['base_dir'], config['store_dir'], config['base_save'], config['train_save']))
    runner(val_dataset, osp.join(config['base_dir'], config['store_dir'], config['base_save'], config['val_save']))
    