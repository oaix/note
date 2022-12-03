Ubuntu有两套网络管理的方法，方法一是用`NetworkManager`，也就是右上角那个有界面的管理器，这个需要用到图形界面`X Window System`，又叫X11或者X ，一些没有X的Linux系统用不了这个，而且这个不能进行一些高级的设置，比如根据IP选择网卡；方法二是修改`/etc/network/interfaces`的脚本设置。这两种方法是冲突的，Ubuntu默认是`NetworkManager`，如果没有`NetworkManager`或者interface被修改则变成 interface



**场景：** 

有两个网卡，有一个网卡A连的网线可以接外网，另一个网卡B接的是局域网，主要用来连接一些硬件设备。如果使用`NetworkManager`会有个问题，不能同时连两个网络。即使通过一些trick连上了，比如先连A，然后打开上网的程序（teamviewer），然后再连上B，但是连着连着teamviewer就会断开，需要重新连A再连B。

**解决办法**

就是将`NetworkManager`禁掉防止冲突，然后设置`/etc/network/interfaces`。

禁掉`NetworkManager`的办法：

```bash
sudo systemctl stop NetworkManager.service # 暂时停止服务，重启后将再次启动
sudo systemctl disable NetworkManager.service # 禁掉服务，重启后也会禁掉
```

设置`/etc/network/interfaces`：


```
# interfaces(5) file used by ifup(8) and ifdown(8)


auto enx30b49eaec725              # 网卡名字，可通过ifconfig命令来查看，每台电脑不一样
iface enx30b49eaec725 inet static # static将IP设置为静态
address 10.10.14.39               # IP地址
network 10.10.14.0                # 掩碼的反例？
netmask 255.255.255.0             # 子网掩码
broadcast 10.10.14.255            # 广播地址，用于同时向网络中所有的工作站进行发送的一个地址，具体看网络相关书籍，我也忘了
gateway 10.10.14.254              # 网关
dns-nameservers 1.2.4.8           # DNS地址，也可以通過/etc/resolv.conf修改
# 设置路由，主机IP为192.168.1.222，192.168.1.20的数据经由10.10.14.254转发，不跑到局域网
up route add -host 192.168.1.222 gw 10.10.14.254
up route add -host 192.168.1.20 gw 10.10.14.254
# 设置路由，所有10.10.0.0网段的数据都经由网关10.10.14.254转发，避免跑到局域网的网卡
up route add -net 10.10.0.0 netmask 255.255.0.0 gw 10.10.14.254


auto enp3s0
iface enp3s0 inet static
address 192.168.1.102
network 192.168.1.0
netmask 255.255.255.0
bradcast 192.168.1.255


auto lo
iface lo inet loopback
```

更详细的参数含义，可以通过`man interfaces`以及`man up route`来查询，其中后者还可以指定端口号

参考：[Linux下route add 命令加入路由列表](https://www.cnblogs.com/gccbuaa/p/7117029.html)

## 针对无线网卡

无线网卡除了以上信息，还需要设置Wifi的名称和密码，并且最好设置成dhcp模式，自动选择IP，而不是static模式：

```
# interfaces(5) file used by ifup(8) and ifdown(8)


auto wlp0s20f3                    # 网卡名字，可通过ifconfig命令来查看，每台电脑不一样
iface wlp0s20f3 inet dhcp         # dhcp模式将自动分配IP，后面IP、子网掩码、网关这些都是不用的，写也无所谓
wpa-ssid my_wifi_name             # 要链接的WiFi名称
wpa-psk my_wifi_passward          # WiFi密码
# address 10.10.14.39             # IP地址不需要填写
network 10.10.14.0                # 掩碼的反例？
netmask 255.255.255.0             # 子网掩码
broadcast 10.10.14.255            # 广播地址，用于同时向网络中所有的工作站进行发送的一个地址，具体看网络相关书籍，我也忘了
gateway 10.10.14.254              # 网关
dns-nameservers 1.2.4.8           # DNS地址，也可以通過/etc/resolv.conf修改
# 设置路由，主机IP为192.168.1.222，192.168.1.20的数据经由10.10.14.254转发，不跑到局域网
up route add -host 192.168.1.222 gw 10.10.14.254
up route add -host 192.168.1.20 gw 10.10.14.254
# 设置路由，所有10.10.0.0网段的数据都经由网关10.10.14.254转发，避免跑到局域网的网卡
up route add -net 10.10.0.0 netmask 255.255.0.0 gw 10.10.14.254


auto enp3s0
iface enp3s0 inet static
address 192.168.1.102
network 192.168.1.0
netmask 255.255.255.0
bradcast 192.168.1.255


auto lo
iface lo inet loopback
```