# Data_frame_transformation

this is to transform imu data from imu link to base link.

## Build
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone
cd ..
catkin build
```

## Run
```bash
roslaunch transform_imu_data transform_data.launch
```

## Config file

`imu_topic`: the imu topic u wanna transform from 

`new_topic`: new topic with the transformed data

`new_frame_id`: frame id of the new topic

`unit_in_g`: true/false
 means the unit of imu's acceleration is in g instead of usual -9.81 ms-2. This is mainly for livox imu.

`extrinsic_T`: [x,y,z], base to imu, units is in **meters**

`extrinsic_R`: [r,p,y], base to imu, units is in **degrees**
