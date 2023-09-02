# GPS KML Extractor for ROS

ROS package for extracting NavSatFix data into a KML file.

This package can be used along with [gps_common](http://ros.org/wiki/gps_common) to extract UTM odometry into a csv file.

As a side bonus, if robot odometry is available, you can extract the data into a csv file alongside.

# Install
No special dependencies, so simply download and `catkin_make`:

```
mkdir -p catkin_ws/src
cd catkin_was/src
git clone https://github.com/tmxkn1/gps_kml_extractor.git
cd ..
catkin_make
```

# Use
To extract GPS data into a KML file only:
```
roslaunch gps_kml_extractor extractor_kml.launch
```
To extract GPS, UTM and robot odom: (note, [gps_common](http://ros.org/wiki/gps_common) is required for UTM)
```
roslaunch gps_kml_extractor extractor_all.launch
```
