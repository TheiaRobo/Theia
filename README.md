Theia
=====

Project for KTH DD2425 2013


What's needed?
------------
* Drivers for PrimeSense (Available on [OpenNI.org](http://www.openni.org/openni-sdk/))
* PCL 1.6 (`sudo apt-get install ros-fuerte-perception-pcl-fuerte-unstable`)


How to compile?
---------------
Just run `rosmake theia`


How to have fun?
----------------
```
roslaunch theia all.launch  
roslaunch openni_launch openni.launch  
roslaunch vision all.launch  
rosrun control_logic brain
```

Tadaaa!


Proudly Presented By
-------
* Diogo Almeida  
* Jimmy Larsson  
* Francisco Martucci  
* Alessandro Motta  
* Clyde Webster