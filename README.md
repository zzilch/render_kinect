Simulation of Kinect Measurements
=============

This C++ project implements the Kinect sensor model as described in 

**BlenSor: Blender Sensor Simulation Toolbox.** *Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang. In Advances in Visual Computing. Lecture Notes in Computer Science. pp 199--208. 2011.*

and as implemented in python as a Blender plugin ([BlenSor Webpage](http://www.blensor.org)).

The simulated measurement shows typical artifacts of a kinect, e.g., occlusion boundaries due to distance between IR projector and IR camera, 8 bit quantisation, smooting within a 9x9 pixel correlation window. This implementation also includes options for adding either Gaussian, Perlin or Simplex noise.

This implementation is simplified in that it accepts only a single rigid object as input.

If you are using this code, please cite our work:

**Robot Arm Pose Estimation through Pixel-Wise Part Classification.** *Jeannette Bohg, Javier Romero, Alexander Herzog and Stefan Schaal. Proceedings of the 2014 IEEE International Conference on Robotics and Automation. pp 3143--3150. 2014.*

Requirements
----------
The following libraries are required to compile the code:

* OpenCV (image I/O, filtering)
* CGAL (Fast Intersection Queries)
* Eigen (Linear Algebra)
* assimp (Mesh I/O)
* noise (Generation of Different Noise Types)

The following libraries are optional:

* OpenMP (Parallelization)
* PCL (Point cloud  I/O)

Compilation
------------

The compilation of this code is tested on Ubuntu (12.04 precise and 12.10 quantal). The CMakeLists.txt file might need to be slightly adapted for correctly linking against PCL dependent on whether you installed it as a standalone library or with ROS.

```
mkdir build
cd build
cmake ..
make -j
```

Testing
------------
There is three small test programs that will render the simulated kinect measurements of a wheel at a number of slightly perturbed transformations relative to the camera.

**render_object**
```
cd bin
./render_object wheel.obj
```
This should store a number of depth and labeled images in ../obj_models If you have PCL installed, it also stores point clouds as pcl files.
![](data/render_object.png?raw=true)  
Point clouds generated from a simulated kinect measurement taken from a wheel in 10 slightly different poses. This measurement does not expose additional noise.
![](data/Wheels.png?raw=true)
  
**render_object_surrounded**
```
cd bin
./render_object_surrounded wheel.obj
```
This should perform virtual scanning from 10 different poses surrounded the wheel.    
![](data/render_object_surrounded.png?raw=true)
  
**render_object_merge**
```
cd bin
./render_object_merge wheel.obj
```
This should perform virtual scanning from 10 different poses by rotating around the X axis.  
![](data/render_object_merge.png?raw=true)
```
pcl_viewer ../obj_models/point_cloud.pcd
```
If you have PCL tools installed, you can visualized the point cloud merged from 10 views by pcl_viewer.  
![](data/point_cloud.png?raw=true)
