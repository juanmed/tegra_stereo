
# GPU-accelerated stereo processing on Tegra X1

This is a ROS implementation of [Embedded real-time stereo estimation via Semi-Global Matching on the GPU](http://www.sciencedirect.com/science/article/pii/S1877050916306561), [D. Hernandez-Juarez](http://www.cvc.uab.es/people/dhernandez/) et al, ICCS 2016.
Code: [https://github.com/dhernandez0/sgm](https://github.com/dhernandez0/sgm)
[D. Hernandez-Juarez](http://www.cvc.uab.es/people/dhernandez/)

It rectifies raw images given a calibrated stereo camera and publishes disparities and pointclouds.

# To build and install on Jetson Nano (with Tegra TX1) and use ZED cameras

## Build and install tegra_stereo package

- Install dependencies:

```bash
sudo apt-get install ros-melodic-image-geometry
sudo apt-get install ros-melodic-image-proc
sudo apt-get install ros-melodic-camera-calibration-parsers
sudo apt-get install ros-melodic-camera-info-manager
sudo apt-get install ros-melodic-cv-bridge
```

You might also need to downgrade opencv to version 3.4.0 (Jetson Nano comes with 4.0.0 preinstalled). To do so follow [this tutorial](https://jkjung-avt.github.io/opencv-on-nano/).

If at any point you get an error similar to this:

```bash
make[2]: *** No rule to make target '/usr/lib/aarch64-linux-gnu/libGL.so', needed by 'lib/libopencv_cudev.so.3.4.6'. Stop.
```

You might be able to solve it with:

```bash
sudo ln -sf libGL.so.1.0.0 /usr/lib/aarch64-linux-gnu/libGL.so
```

Finally build the package with `catkin build`.

## Install zed-ros-wrapper

Follow the [official instructions](https://www.stereolabs.com/docs/ros/) for installing zed-ros-wrapper.

## Execute

- Run

Open one terminal and do:

```bash
roslaunch tegra_stereo zedm_launch.launch
```

Open another terminal and do:
```bash
roslaunch tegra_stereo tegra_stereo.launch
```

You can subscribe to `/tegra_stereo/depth/image` or `/tegra_stereo/points` to get the depth or point cloud images.


