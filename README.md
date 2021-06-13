# Sensor Fusion Nano Degree

![ObstacleDetectionFPS](https://user-images.githubusercontent.com/68550704/121780072-fcb62980-cb9e-11eb-8f27-f16e22c2baa5.gif)


### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

# RANSAC Algorithm for Segmentation

PCL library has preefined function called "segmentplane" for point cloud segmentation but in this project we developed this algorithm on our own intially for 2d and then later we extended it to 3d for the final project.

RANSAC stands for Random Sample Consensus, and is a method for detecting outliers in data. RANSAC runs for a max number of iterations, and returns the model with the best fit. Each iteration randomly picks a subsample of the data and fits a model through it, such as a line or a plane. Then the iteration with the highest number of inliers or the lowest noise is used as the best model.

![ransac](https://user-images.githubusercontent.com/68550704/121816623-fac29800-cc7c-11eb-8852-3d2fb3d05594.gif)

One type of RANSAC version selects the smallest possible subset of points to fit. For a line, that would be two points, and for a plane three points. Then the number of inliers are counted, by iterating through every remaining point and calculating its distance to the model. The points that are within a certain distance to the model are counted as inliers. The iteration that has the highest number of inliers is then the best model. This will be the version that you will implement in this quiz.

Other methods of RANSAC could sample some percentage of the model points, for example 20% of the total points, and then fit a line to that. Then the error of that line is calculated, and the iteration with the lowest error is the best model. This method might have some advantages since not every point at each iteration needs to be considered. It’s good to experiment with different approaches and time results to see what works best.

# KD Tree Algorithm for clustering Point cloud

Now we have segmented the points and we can recognize which ones represent obstacles for your car. It would be great to break up and group those obstacle points, especially if you want to do multiple object tracking with cars, pedestrians, and bicyclists, for instance. One way to do that grouping and cluster point cloud data is called euclidean clustering.

So even for this we developed the algorithm in 2D intially and later extending it to 3d for implementing in final project.so lets divide this task in to 2 steps

* Inserting points in to KD tree(2D)
* Searching points in KD Tree(2D)

# Inserting points in to KD tree(2D)

A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering.

## Workflow

* The points which is inserted first will become the root of the tree
* Later inserted points will be allotted to the left if its 'x' value is lesser than root 'x' value or to the right if greater
* when d%2 is equal to 0 x split will occur and at 1 y split will occur.Here d means the number of dimensions we are working in this case 2.
* consecutive points will be compared starting from root and based on depth x or y will be compared and alloted to the repective position in kd tree

![kdtree5](https://user-images.githubusercontent.com/68550704/121817135-c3a1b600-cc7f-11eb-8931-e13203da08bf.png)

                                      Initial 2d points

![2dpoints](https://user-images.githubusercontent.com/68550704/121817188-1a0ef480-cc80-11eb-8e34-f6ecf1442627.png)

              KD tree formed

![Kdtree_insert](https://user-images.githubusercontent.com/68550704/121817175-02377080-cc80-11eb-8695-1aa8477f1362.png)

# Searching points in KD Tree(2D)

Once points are able to be inserted into the tree, the next step is being able to search for nearby points inside the tree compared to a given target point. Points within a distance of distanceTol are considered to be nearby. The KD-Tree is able to split regions and allows certain regions to be completely ruled out, speeding up the process of finding nearby neighbors.

The naive approach of finding nearby neighbors is to go through every single point in the tree and compare their distances with the target, selecting point indices that fall within the distance tolerance of the target. Instead with the KD-Tree you can compare distance within a boxed square that is 2 x distanceTol for length, centered around the target point. If the current node point is within this box then you can directly calculate the distance and see if the point id should be added to the list of nearby ids. Then you see if your box crosses over the node division region and if it does compare that next node. You do this recursively, with the advantage being that if the box region is not inside some division region you completely skip that branch.

![Kdtree_search](https://user-images.githubusercontent.com/68550704/121817614-c18d2680-cc82-11eb-90e0-629dac7bb701.gif)


Results will look like this.Here the three colours indicates three clusters.

<img width="286" alt="Kdtree_cluster" src="https://user-images.githubusercontent.com/68550704/121817646-ed101100-cc82-11eb-99f7-6f0c5719495e.png">


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php

