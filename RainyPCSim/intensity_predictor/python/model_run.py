import sys

import torch.utils.data as data
import os
import inten
import otils as ot
import torchutils as tu
import datetime
import shutil
if __name__ == '__main__':
    config = ot.io.load_multi_yml(sys.argv[1])
    config['store_dir'] = config['store_dir'] + '_' + datetime.datetime.today().strftime('%Y%m%d%H%M%S')
    os.makedirs(os.path.join(config['base_dir'], config['store_dir']), exist_ok=True)
    shutil.copy2(sys.argv[1], os.path.join(config['base_dir'], config['store_dir'], os.path.basename(sys.argv[1])))
    print(config['base_dir'], config['store_dir'])
    if 'seed' in config:
        tu.seed_all(config['seed'])
    runner = inten.data.Runner(config)
    trn_dataset = data.DataLoader(inten.data.Dataset(config['train']), **config['train_loader'])
    val_dataset = data.DataLoader(inten.data.Dataset(config['val']), **config['val_loader'])
    if 'scheduler' in config:
        scheduler = inten.utils.scheduler(config['scheduler'], runner.optimizer)
        runner.scheduler = scheduler
    else:
        scheduler = None
    for epoch_id in range(config['epochs']):
        trn_loss = runner(trn_dataset, tu.TorchMode.TRAIN)
        val_loss = runner(val_dataset, tu.TorchMode.EVAL)
        if scheduler is not None:
            scheduler.step()
            print('Epoch {} current lr:'.format(epoch_id), scheduler.get_last_lr())
