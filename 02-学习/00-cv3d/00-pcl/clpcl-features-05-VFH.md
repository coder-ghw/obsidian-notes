
[VFH signatures](https://pcl.readthedocs.io/projects/tutorials/en/master/vfh_estimation.html#vfh-estimation)

This document describes the Viewpoint Feature Histogram descriptor, a novel representation for point clusters for the problem of Cluster Recognition and 6DOF Pose Estimation.

## 理论基础

The VFH has its roots in the FPFH descriptor. Due to its speed and discriminatve power, we decide to
leverage the strong recogmition results of FPFH, but to add in **viewpoint variance** while retaining invariance to scale.

我们对物体识别和姿态识别问题的贡献是扩展了FPFH算法，使其能够估计整个物体群集（如下图所示），并计算每个点处估计的法线与视点方向之间的额外统计信息。为此，我们使用了将视点方向直接混合到FPFH中相对法线角度计算的关键思想。

![[Pasted image 20230608195531.png|800]]

视点组件是通过收集每个法线与视点方向所成角度的直方图来计算的。请注意，我们不是指对每个法线的视角，因为这样不具有尺度不变性，而是指中心视点方向转换到每个法线上所形成的角度。第二个组件测量了相对于中心点处观察方向和表面上每个法线之间的平移、倾斜和偏航角度，如快速点特征直方图（FPFH）描述符所述。

![[Pasted image 20230608195838.png|800]]

The new assembled feature is therefore called the Viewpoint Feature Histogram (VFH). The figure below presents this idea with the new feature consisting of two parts:

> 1. a viewpoint direction component and
>     
> 2. a surface shape component comprised of an extended FPFH.

## Code

The major difference between the PFH/FPFH descriptors and VFH, is that for a given point cloud dataset, only a single VFH descriptor will be estimated, while the resultant PFH/FPFH data will have the same number of entries as the number of points in the cloud.

```cpp
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

  ... read, pass in or create a point cloud with normals ...
  ... (note: you can create a single PointCloud<PointNormal> if you want) ...

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (normals);
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);

  // vfhs->size () should be of size 1*
}
```