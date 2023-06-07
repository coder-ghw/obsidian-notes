
```cpp
#include <pcl/visualizer.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

int main()
{
  // 1. load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile("", *cloud);

  // 2. estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::InegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.f);
  ne.setInputCloud(cloud);
  ne.compute(*normal);

  // 3. visualize normals
  pcl::visualization::PCLVisualizer viewer("pcl viewer");
  viewer.setBackgroudColor(0.f, 0.f, 0.5f);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

  while(!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
}
```
The following normal estimation methods are available:
```cpp
enum NormalEstimationMethod
{
  COVARIANCE_MATRIX,
  AVERAGE_3D_GRADIENT,
  AVERAGE_DEPTH_CHANGE
};
```

- The COVARIANCE_MATRIX mode creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood.
- The AVERAGE_3D_GRADIENT mode creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients.
- The AVERAGE_DEPTH_CHANGE mode creates only a single integral image and computes the normals from the average depth changes.

