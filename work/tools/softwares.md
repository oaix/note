# Useful Solftware

## Google Drive-command-line
This [PPA](https://launchpad.net/~twodopeshaggy/+archive/ubuntu/drive) includes Ubuntu, Mint, Linux Lite, etc. [List](http://distrowatch.com/search.php?basedon=Ubuntu)

```sh
sudo add-apt-repository ppa:twodopeshaggy/drive
sudo apt-get update
sudo apt-get install drive
```

### [initializing](https://github.com/odeke-em/drive#installation)
```sh
drive init ~/gdrive
cd ~/gdrive
drive deinit # the opposite of drive init
drive pull (-force)
drive push
```



## ffmpeg图像视频

1. 将图片合成视频

   ```sh
   ffmpeg -framerate 1 -i %02d.png -c:v libx264 -pix_fmt yuv420p output.mp4
   
   ffmpeg -framerate 30 -pattern_type glob -i './*.jpeg' -c:v libx264 -pix_fmt yuv420p output.mp4
   ```
   
   
