from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')
 
info = lidar.get_info()
print(info)
 
health = lidar.get_health()
print(health)
 
try:
    for i, scan in enumerate(lidar.iter_scans()):
        print('%d: Got %d measurments' % (i, len(scan)))
        
except KeyboardInterrupt:  
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()