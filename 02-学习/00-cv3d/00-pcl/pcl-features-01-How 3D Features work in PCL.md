
https://pcl.readthedocs.io/projects/tutorials/en/master/how_features_work.html#how-3d-features-work

## 点特征的属性

- **rigid transformations** - _that is, 3D rotations and 3D translations in the data should not influence the resultant feature vector F estimation;_
    
- **varying sampling density** - _in principle, a local surface patch sampled more or less densely should have the same feature vector signature;_
    
- **noise** - _the point feature representation must retain the same or very similar values in its feature vector in the presence of mild noise in the data.

## 怎么输入数据

- an entire point cloud dataset, given via **setInputCloud (PointCloudConstPtr &)** - **mandatory**
- a subset of a point cloud dataset, given via **setInputCloud (PointCloudConstPtr &)** and **setIndices (IndicesConstPtr &)** - **optional**
- in addition, the set of point neighbors to be used, can be specified through an additional call, **setSearchSurface (PointCloudConstPtr &)**. This call is optional, and when the search surface is not given, the input point cloud dataset is used instead by default.
- the image below presents all four cases:

![[Pasted image 20230602145252.png]]

The most useful example when **setSearchSurface()** should be used, is when we have a very dense input dataset, but we do not want to estimate features at all the points in it, but rather at some keypoints discovered using the methods in pcl_keypoints,or
at a soensampled vesion os the cloud. in this case we pass the downsampled input via setInputCloud(), and the original data as set SearchSurfase. ## 法线估计例子

The following code snippet will estimate a set of surface normals for all the points in the input dataset.

```cpp
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
  //... read, pass in or create a point cloud...
  
  //create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (kdtree);

  // output dataset
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*cloud_normals);
}
```

the following code snippet will estimate a set of surface normals for all the points in the input dataset, but will estimate their nearest neighbors using another dataset. As previously mentioned, a good usecase for this is when the input is a downsampled version of the surface.
```cpp
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

  //... read, pass in or create a point cloud ...

  //... create a downsampled version of it ...

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_downsampled);

  // Pass the original data (before downsampling) as the search surface
  ne.setSearchSurface (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given surface dataset.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->size () should have the same size as the input cloud_downsampled->size ()
}
```