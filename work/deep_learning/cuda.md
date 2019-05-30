### [Install](https://www.pugetsystems.com/labs/hpc/How-To-Install-CUDA-10-together-with-9-2-on-Ubuntu-18-04-with-support-for-NVIDIA-20XX-Turing-GPUs-1236/)
![fc4fda5a](images/fc4fda5a.png)

### [install nvidia driver](https://gist.github.com/wangruohui/df039f0dc434d6486f5d4d098aa52d07#download-the-driver)
```sh
sudo apt-get purge nvidia-\*
# Recommended if .deb files from NVIDIA were installed
# Change 1404 to the exact system version or use tab autocompletion
# After executing this file, /etc/apt/sources.list.d should contain no files related to nvidia or cuda
sudo dpkg -P cuda-repo-ubuntu1604

#Create a file at /etc/modprobe.d/blacklist-nouveau.conf with the following contents:
blacklist nouveau
options nouveau modeset=0

sudo update-initramfs -u
sudo reboot
Ctl + Alt + F2
sudo service lightdm stop
chmod +x NVIDIA-Linux-x86_64-384.69.run
sudo ./NVIDIA-Linux-x86_64-384.69.run --dkms -s
```