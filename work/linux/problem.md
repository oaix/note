//my.oschina.net/renwofei423/blog/635798)
+ 字体修复
  解压该文件运行`./font.sh`
  [wps_symbol_fonts.tar.gz](images/a783cdb9.gz)

+ [无法输入中文](https://blog.csdn.net/wf19930209/article/details/78481589)
```sh
#! /bin/bash
#------------------------------------------------------------------------------
# Filename:    chineseInputForWPS.sh
# Usage:       ./chineseInputForWPS.sh
# Version:     1.0
# Date:        2017-31-05
# Author:      vincent
# Email:       N/A
# Description: N/A
# Notes:       N/A
#-------------------------------------------------------------------------------

declare XMODIFIERS='export XMODIFIERS="@im=fcitx"'
declare QT_IM_MODULE='export QT_IM_MODULE="fcitx"'
declare wpsPath="/usr/bin/wps"
declare wppPath="/usr/bin/wpp"
declare etPath="/usr/bin/et"

outputMsg()
{
    if [ $1 -ne 0 ]
    then
        echo "unsuccessful !!"
        exit
    fi
}
sudo sed -i "2i$XMODIFIERS" $wpsPath
outputMsg $?
sudo sed -i "2i$QT_IM_MODULE" $wpsPath
outputMsg $?

sudo sed -i "2i$XMODIFIERS" $wppPath
outputMsg $?
sudo sed -i "2i$QT_IM_MODULE" $wppPath
outputMsg $?

sudo sed -i "2i$XMODIFIERS" $etPath
outputMsg $?
sudo sed -i "2i$QT_IM_MODULE" $etPath
outputMsg $?

echo "Successful! You can do it!"

exit 0

```

## [将launcher放置在屏幕下方](https://www.howtogeek.com/251616/how-to-move-the-unity-desktops-launcher-to-the-bottom-of-your-screen-on-ubuntu-16.04/)
```
gsettings set com.canonical.Unity.Launcher launcher-position Bottom
```

+ [硬盘自动挂载](https://www.jianshu.com/p/ec5579ef15a6)

[permissions - How do I use 'chmod' on an NTFS (or FAT32) partition? - Ask Ubuntu](https://askubuntu.com/questions/11840/how-do-i-use-chmod-on-an-ntfs-or-fat32-partition)
`<file system> <mount point>   <type>  <options>       <dump>  <pass>`

`UUID=5782ABAE11211E6E /media/robosense            ntfs    defaults              0       0
`
(1) 分区设备文件名或UUID
查看所有分区设备的UUID：
`ls -l /dev/disk/by-uuid/ `
(2) 挂载点
(3) 文件系统名称
(4) 挂载参数，挂载权限
(5) 指定分区是否被dump备份，0代表不备份，1代表每天备份，2代表不定期备份。
(6) 指定分区是否被fsck检测，0代表不检测，其他数字代表检测的优先级，比如1的优先级比2高。根目录所在的分区的优先级为1，其他分区的优先级为大于或等于2

+ 设置mtu
  [sudo ethtool -s em1 autoneg off speed 100 duplex full](https://www.garron.me/en/linux/ubuntu-network-speed-duplex-lan.html)
  The MTU is specified in terms of bytes or octets of the largest protocol data unit that the layer can pass onwards. ... For example, with Ethernet, the maximum frame size is 1518 bytes, 18 bytes of which are overhead (header and FCS), resulting in an MTU of 1500 bytes.
  (1) 使用命令设置，但是每次重启后需要重新设置
```sh
ifconfig ${Interface} mtu ${SIZE} up
ifconfig eth1 mtu 9000 up
```
[Note this will only work if supported by both the network nterface card and the network components such as switch.](https://www.cyberciti.biz/faq/centos-rhel-redhat-fedora-debian-linux-mtu-size/)
(2)将特定的ip链接的mtu值设为期望值，可永久有效
直接在`Edit Connections ...`中修改相应链接的mtu值，保存以后就可以不用再设置了。
![](images/533ccead.png)


一般电脑支持mtu值为9000,有些配置好一些的电脑能设置为9200
(3)[将设置写入系统文件dhclient.conf](https://stackoverflow.com/questions/26179699/how-to-change-mtu-setting-permanently)中，可永久有效，但是自动ip链接时会出现问题
将一下内容写入/etc/dhcp/dhclient.conf

```
default interface-mtu 9000;
supercede interface-mtu 9000;
```
(4)将对应网段的mtu设为期望值
[microHOWTO: Change the MTU of an network interface using DHCP](http://www.microhowto.info/howto/change_the_mtu_of_a_network_interface_using_dhcp.html)
我们lidar的网段为192.168.1.0
/etc/dhcp/dhclient.conf

```
subnet 192.168.1.0 netmask 255.255.255.0 {
  option broadcast-address 192.168.0.255;
  option interface-mtu 9000;
}
```

+ [How do I use 'chmod' on an NTFS (or FAT32) partition](https://askubuntu.com/questions/11840/how-do-i-use-chmod-on-an-ntfs-or-fat32-partition)改变ntfs文件系统中文件和路径的权限
  I have a script that I need to execute on an NTFS partition. The script's permission is set to 600.
  I attempted to modify the permissions by running chmod 755 script.sh, which doesn't report a failure or anything - but it also doesn't change the permissions on the file。
  The mode is determined by the partition's mount options (you cannot change it via chmod).其权限是挂载时确定的，不能使用`chmod`命令来修改。

+ [uninstall `make install` software](https://stackoverflow.com/questions/1439950/whats-the-opposite-of-make-install-i-e-how-do-you-uninstall-a-library-in-li)
  (1)如果比较幸运，软件作者提供了uninstall命令，那么：
```sh
sudo make uninstall
```
(2)如果有`install_manifest.txt`，该文件在`build`文件夹下，列出了安装到系统的文件清单，直接将这些文件删除即可。