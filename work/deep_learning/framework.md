# Framework

## TensorFlow

### [install](https://medium.com/@yckim/tensorflow-1-3-install-on-ubuntu-16-04-2d191a6e5546)

```sh
sudo pip install tensorflow-gpu==1.3.0
pip install --robosense install tensorflow==1.3.0
```



## [PyTorch](https://pytorch.org/docs/stable/torch.html#torch.arange)

推荐使用conda安装。

### [tensorboardX可视化](https://tensorboardx.readthedocs.io/en/latest/tensorboard.html)

```python
from tensorboardX import SummaryWriter
writer = SummaryWriter(logdir=root_directory + '/log', comment='L3Net')
writer.add_scalar('train_loss', loss, epoch) # writer.add_scalar('myscalar', value, iteration)

with writer:
    pc_map_index_new_0 = np.random.randint(0, 1000, 128*64*1331)
    pc_map_index_new_00 = torch.from_numpy(pc_map_index_new_0).unsqueeze(0)
    arr_map0 = torch.randn(1, 1100, 4)
    arr_online0 = torch.randn(1, 128*64, 4)
    label0 = torch.randn(1, 1, 3)
    if use_gpu:
        pc_map_index_new_00, arr_map0, arr_online0 = pc_map_index_new_00.to(gpu), arr_map0.to(gpu), arr_online0.to(gpu)
    writer.add_graph(model, (pc_map_index_new_00, arr_map0, arr_online0))


writer.close()
```



### [save model](https://stackoverflow.com/questions/42703500/best-way-to-save-a-trained-model-in-pytorch)

https://github.com/pytorch/tutorials/blob/master/beginner_source/saving_loading_models.py

https://pytorch.org/tutorials/beginner/saving_loading_models.html

```python
import os

import torch
import torch.nn as nn

'''
Notes:
# save and load entire model
torch.save(model, "model.pth")
model = torch.load("model.pth")
# save and load only the model parameters(recommended)
torch.save(model.state_dict(), "params.pth")
model.load_state_dict(torch.load("params.pth"))
'''
__all__ = ["CheckPoint"]

class CheckPoint(object):
    '''
    save model state to file
    check_point_params: model, optimizer, epoch
    '''
    def __init__(self, save_path):
        '''initialize class
        Arguments:
            save_path {string} -- path to save files
        '''
        self.save_path = os.path.join(save_path, "check_point")
        # make directory
        if not os.path.isdir(self.save_path):
            print ">>> Path not exists, create path {}".format(self.save_path)
            os.makedirs(self.save_path)

    def load_state_dict(self, model, pth_file):
        '''load state dict from file

        Arguments:
            model {nn.Module} -- target model
            pth_file {string} -- path of the saved state_dict

        Returns:
            [nn.Module] -- new model
        '''

        model.eval()
        state_dict = torch.load(pth_file)
        model.load_state_dict(state_dict)
        return model

    def load_checkpoint(self, checkpoint_path):
        '''load checkpoint file

        Arguments:
            checkpoint_path {string} -- path to the checkpoint file

        Returns:
            model_state_dict {dict} -- state dict of model
            optimizer_state_dict {dict} -- state dict of optimizer
            epoch {integer} -- number of epoch
        '''

        if os.path.isfile(checkpoint_path):
            print ">>> Load resume check-point from:", checkpoint_path
            self.check_point_params = torch.load(checkpoint_path)
            model_state_dict = self.check_point_params['model']
            optimizer_state_dict = self.check_point_params['optimizer']
            extra_params = self.check_point_params['extra_params']
            # because we save the index of lastest training/testing epochs,
            # we need to add 1 to start a new epoch.
            epoch = self.check_point_params['epoch']
            return model_state_dict, optimizer_state_dict, epoch
        else:
            assert False, "file not exits" + checkpoint_path

    def save_checkpoint(self, model, optimizer, epoch, name, extra_params=None):
        '''save checkpoint data of target model and optimizer

        Arguments:
            model {torch.nn.Module} -- model
            optimizer {torch.nn.optimizer} -- optimizer
            epoch {integer} -- index of training/testing epochs
            name {string} -- name of pth file

        Note: if we add hook to the grad by using register_hook(hook), then the hook function
        can not be saved so we need to save state_dict() only. Although save state dictionary
        is recommended, some times we still need to save the whole model as it can save all
        the information of the trained model, and we do not need to create a new network in
        next time. However, the GPU information will be saved too, which leads to some issues
        when we use the model on different machine

        this function will generate a checkpoint file, which contains a dict
        {
            'model': state dict of model
            'optimizer': state dict of optimizer
            'epoch': index of training epochs
            'extra_params': extra params to save
        }
        '''

        self.check_point_params = {'model': None,
                                   'optimizer': None,
                                   'epoch': None,
                                   'extra_params': None}
        model.eval()
        # get state_dict from model and optimizer
        if isinstance(model, nn.DataParallel):
            model_state_dict = model.module.state_dict()
        else:
            model_state_dict = model.state_dict()

        optimizer_state_dict = optimizer.state_dict()

        # save information to a dict
        self.check_point_params['model'] = model_state_dict
        self.check_point_params['optimizer'] = optimizer_state_dict
        self.check_point_params['epoch'] = epoch + \
            1  # add 1 to start a new epoch
        self.check_point_params['extra_params'] = extra_params

        # save to file
        torch.save(self.check_point_params, os.path.join(
            self.save_path, name))

    def save_state_dict(self, model, name):
        '''save state dict of model

        Arguments:
            model {torch.nn.Module} -- model to save
            name {bool} -- name of saved file
        '''

        # get state dict
        if isinstance(model, nn.DataParallel):
            model_state_dict = model.module.state_dict()
        else:
            model_state_dict = model.state_dict()
        torch.save(model_state_dict, os.path.join(self.save_path, name))
```

#### Case # 1: Save the model to use it yourself for inference

You save the model, you restore it, and then you change the model to evaluation mode. This is done because you usually have `BatchNorm`and `Dropout` layers that by default are in train mode on construction:

```python
torch.save(model.state_dict(), filepath)

#Later to restore:
model.load_state_dict(torch.load(filepath))
model.eval()
# Remember that you must call ``model.eval()`` to set dropout and batch
# normalization layers to evaluation mode before running inference.
# Failing to do this will yield inconsistent inference results.
```

#### Case # 2: Save model to resume training later

If you need to keep training the model that you are about to save, you need to save more than just the model. You also need to save the state of the optimizer, epochs, score, etc. You would do it like this:

```python
state = {
    'epoch': epoch,
    'state_dict': model.state_dict(),
    'optimizer': optimizer.state_dict(),
    ...
}
torch.save(state, filepath)


#    model = TheModelClass(*args, **kwargs)
#    optimizer = TheOptimizerClass(*args, **kwargs)
#
#    checkpoint = torch.load(PATH)
#    model.load_state_dict(checkpoint['model_state_dict'])
#    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
#    epoch = checkpoint['epoch']
#    loss = checkpoint['loss']

# If you
# wish to resuming training, call ``model.train()`` to ensure these layers
# are in training mode.
```

#### Case # 3: Model to be used by someone else with no access to your code

In Tensorflow you can create a `.pb` file that defines both the architecture and the weights of the model. This is very handy, specially when using `Tensorflow serve`. The equivalent way to do this in Pytorch would be:

```python
torch.save(model, filepath)

# Then later:
model = torch.load(filepath)
```



### [构建模型的几种方法](https://www.cnblogs.com/denny402/p/7593301.html)

```python
import torch
import torch.nn.functional as F
from collections import OrderedDict

# Method 1 -----------------------------------------

class Net1(torch.nn.Module):
    def __init__(self):
        super(Net1, self).__init__()
        self.conv1 = torch.nn.Conv2d(3, 32, 3, 1, 1)
        self.dense1 = torch.nn.Linear(32 * 3 * 3, 128)
        self.dense2 = torch.nn.Linear(128, 10)

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.conv(x)), 2)
        x = x.view(x.size(0), -1)
        x = F.relu(self.dense1(x))
        x = self.dense2()
        return x

print("Method 1:")
model1 = Net1()
print(model1)


# Method 2 ------------------------------------------
class Net2(torch.nn.Module):
    def __init__(self):
        super(Net2, self).__init__()
        self.conv = torch.nn.Sequential(
            torch.nn.Conv2d(3, 32, 3, 1, 1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(2))
        self.dense = torch.nn.Sequential(
            torch.nn.Linear(32 * 3 * 3, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 10)
        )

    def forward(self, x):
        conv_out = self.conv1(x)
        res = conv_out.view(conv_out.size(0), -1)
        out = self.dense(res)
        return out

print("Method 2:")
model2 = Net2()
print(model2)


# Method 3 -------------------------------
class Net3(torch.nn.Module):
    def __init__(self):
        super(Net3, self).__init__()
        self.conv=torch.nn.Sequential()
        self.conv.add_module("conv1",torch.nn.Conv2d(3, 32, 3, 1, 1))
        self.conv.add_module("relu1",torch.nn.ReLU())
        self.conv.add_module("pool1",torch.nn.MaxPool2d(2))
        self.dense = torch.nn.Sequential()
        self.dense.add_module("dense1",torch.nn.Linear(32 * 3 * 3, 128))
        self.dense.add_module("relu2",torch.nn.ReLU())
        self.dense.add_module("dense2",torch.nn.Linear(128, 10))

    def forward(self, x):
        conv_out = self.conv1(x)
        res = conv_out.view(conv_out.size(0), -1)
        out = self.dense(res)
        return out

print("Method 3:")
model3 = Net3()
print(model3)



# Method 4 ------------------------------------------
class Net4(torch.nn.Module):
    def __init__(self):
        super(Net4, self).__init__()
        self.conv = torch.nn.Sequential(
            OrderedDict(
                [
                    ("conv1", torch.nn.Conv2d(3, 32, 3, 1, 1)),
                    ("relu1", torch.nn.ReLU()),
                    ("pool", torch.nn.MaxPool2d(2))
                ]
            ))

        self.dense = torch.nn.Sequential(
            OrderedDict([
                ("dense1", torch.nn.Linear(32 * 3 * 3, 128)),
                ("relu2", torch.nn.ReLU()),
                ("dense2", torch.nn.Linear(128, 10))
            ])
        )

    def forward(self, x):
        conv_out = self.conv1(x)
        res = conv_out.view(conv_out.size(0), -1)
        out = self.dense(res)
        return out

print("Method 4:")
model4 = Net4()
print(model4)
```

