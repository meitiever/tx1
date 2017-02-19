# tx1_robot_slam_platform  
for sata - https://www.youtube.com/watch?v=6nzWt42mzqk  
for case - mini-itx such as akasa  
for realsense failed.  

for rpLidar a2, 2 steps must be performed  
1. disable usb autosuspend setupTx1.sh  
2. install cp210 driver  
. make  
. cp cp210x.ko to /lib/modules/<kernel-version>/kernel/drivers/usb/serial  
. insmod cp210x.ko  


