
[features-PFH](https://pcl.readthedocs.io/projects/tutorials/en/master/pfh_estimation.html#pfh-estimation)

As point feature representations go, surface normals and curvature estimates are somewhat basic in their representations of the geomotry around a specific point. Though extremely fast and eeasy to compute, they cannot capture too much detail.as they approximate the geometry of a point’s k-neighborhood with only a few values. As a direct consequence, most scenes will contain many points with the same or very similar feature values, thus reducing their informative characteristics.

## Theoretical primer

The goal of PFH formulation is to encode a point,s k-neiborhood geometrical properties by generalizing the mean curvature zround the point using a multi-demensional histogram of values. this highly dimensional hyperspace provides an informative signature for the feature reperesentation, is invariant to the 6D pose of the underlying surface, and copes very well with different sampling decsities or noise levels present in the neighborhood.

A Point Feature Histogram representation is based on the relationships between the points in the k-neighborhood and their estimated surface normals. Simply put, it attempts to capture as best as possible the sampled surface variations by taking into account all the interactions between the directions of the estimated normals. **The resultant hyperspace is thus dependent on the quality of the surface normal estimations at each point.**

![[Pasted image 20230606102804.png|500]]

To compute the relative difference between two points $p_i$ and $p_j$ and their associated normals $n_i$ and $n_j$, we define a fixed coordinate frame at one of the points(see the figure below)

![[Pasted image 20230606102839.png|800]]

Using the above UVW frame, the difference between the two normals $n_s$ and $n_t$ can be expressed as a set of angular features as follows:

![[Pasted image 20230606103503.png|900]]


To create the final PFH representation for the query point, the set of all quadruplets is binned into a histogram. The binning process divides each feature’s value range into **b** subdivisions, and counts the number of occurrences in each subinterval. Since three out of the four features presented above are measures of the angles between normals, their values can easily be normalized to the same interval on the trigonometric circle. A binning example is to divide each feature interval into the same number of equal parts, and therefore create a histogram with $b^4$ bins in a fully correlated space. In this space, a histogram bin increment corresponds to a point having certain values for all its 4 features. The figure below presents examples of Point Feature Histograms representations for different points in a cloud.

In some cases, the fourth feature, **d**, does not present an extreme significance for 2.5D datasets, usually acquired in robotics, as the distance between neighboring points increases from the viewpoint. Therefore, omitting **d** for scans where the local point density influences this feature dimension has proved to be beneficial.

## Code for PFH features

The default PFH implementation uses 5 binning subdivisions (e.g., each of the four feature values will use this many bins from its value interval), and does not include the distances (as explained above – although the **computePairFeatures** method can be called by the user to obtain the distances too, if desired) which results in a 125-byte array (![5^3](https://pcl.readthedocs.io/projects/tutorials/en/master/_images/math/528b6b269b86cf640f706bfcd5a3c9a84dc52d58.png)) of float values. These are stored in a **pcl::PFHSignature125** point type.

```cpp
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCLoud<pcl::Normal>());

  // read and load data

  // create the pfh estimate class and pass the input data
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
  pfh.setInputCloud(cloud);
  pfh.setInputNormals(normals);

  // create an empty kdtree and pass it to pfh class
  pcl::search::kdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::kdTree<pcl::PointXYZ>());
  pfh.setSeatchMethod(tree);

  // result
  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());
  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  pfh.setRadiusSearch (0.05);

  // compute
  pfh.compute (*pfhs);
}
```

其中pcl::PFHSignature125为直方图输出结果类型，125的含义是 这个类中特征选取了三个角度值，而没有距离信息（需要距离信息需要computePairFeatures方法计算），且角度在构建直方图时默认采用了5个间隔(bin)，通俗来讲就是 有三个角度，每个角度有5份，穷举可能出现的结果构建直方图，即每个角度选一份出来组成一块，排列组合A15 * A15 * A15 即文档中所说的5 * 5 * 5=5^3

![[Pasted image 20230608192945.png|600]]


### FPFH

相比PFH，FPFH为了计算快速，计算时没有使用邻域网络交叉计算，而是通过邻域各点的SPFH加权得到结果。SPFH即针对某点，只计算此点与邻域间的特征角度和距离信息，不必计算邻域内点间的特征信息。
```cpp
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

  ... read, pass in or create a point cloud with normals ...
  ... (note: you can create a single PointCloud<PointNormal> if you want) ...

  // FPFH估计类，其中pcl::FPFHSignature33为输出类型为直方图数据
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud);
  fpfh.setInputNormals (normals);
 
  //以下类似于FPH代码
  pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
  fpfh.setSearchMethod (tree);

 //输出结果
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  fpfh.setRadiusSearch (0.05);

  fpfh.compute (*fpfhs);

  // fpfhs->size () should have the same size as the input cloud->size ()*
}
```

- 其中`pcl::FPFHSignature33`，33的含义是3个角度，默认分割11份，不像PFH125那样穷举，而是直接独立拼凑到一起，即11+11+11=33
