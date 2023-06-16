
## 怎么选择使用哪种重建算法？

CGAL 提供了3种不同的重建算法: 泊松重建，前进法，尺度空间法
- Poisson Surface Reconstruction
- Advancing Front Surface Reconstruction
- Scale Space Surface Reconstruction

由于重建是一个不适定问题，必须通过先验知识进行正则化。 先验的差异导致不同的算法，并且选择其中一种方法取决于这些先验。 例如，Poisson始终生成封闭形状（包围体积）并需要法线，但不插值输入点（输出表面未完全通过输入点）。 以下表格列出了输入和输出的不同属性，以帮助用户选择最适合每个问题的方法：

|条件 |Poisson|Advancing front|Scale space|
|---|:-:|:-:|:-:|
|Are normals required?|Yes|No|No|
|Is noise handled?|Yes|By preprocessing|Yes|
|Is variable sampling handled?|Yes|Yes|By preprocessing|
|Are input points exactly on the surface?|No|Yes|Yes|
|Is the output always closed?|Yes|No|No|
|Is the output always smooth?|Yes|No|No|
|Is the output always manifold?|Yes|Yes|Optional|
|Is the output always orientable?|Yes|Yes|Optional|

下图展示了使用CGAL工具进行常见重建步骤的概览:

![[Pasted image 20230614194916.png|1200]]

下面详细展开每个模块进行介绍：

## IO read
支持 ply xyz las格式的数据
```cpp
Point_set points;

std::string fname = argc==1?[CGAL::data_file_path] : argv[1];

if (argc < 2)
{
  std::cerr << "Usage: " << argv[0] << " [input.xyz/off/ply/las]" << std::endl;
  std::cerr <<"Running " << argv[0] << " data/kitten.xyz -1\n";
}

std::ifstream stream (fname, std::ios_base::binary);
if (!stream)
{
  std::cerr << "Error: cannot read file " << fname << std::endl;
  return EXIT_FAILURE;
}
stream >> points;
std::cout << "Read " << points.size () << " point(s)" << std::endl;
if (points.empty())
  return EXIT_FAILURE;
```
## Preprocessing

预处理操作是可选的 if the input point has no imperfections

- 去除飞点
```cpp
  
typename Point_set::iterator rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>(
							points,
                            24, // Number of neighbors considered for evaluation
                            points.parameters().threshold_percent (5.0)); // Percentage of points to remove

points.remove(rout_it, points.end());

std::cout << points.number_of_removed_points() << " point(s) are outliers." << std::endl;
// Applying point set processing algorithm to a CGAL::Point_set_3
// object does not erase the points from memory but place them in
// the garbage of the object: memory can be freeed by the user.
points.collect_garbage();

```

- 简化
一些激光扫描仪生成的点具有广泛变化的采样率。通常，扫描线非常密集地采样，但两条扫描线之间的间隙要大得多，导致点云过于庞大且采样密度差异很大。这种类型的输入点云可能会使用算法生成不完美的输出结果，因为通常只能处理小范围内采样密度变化。
```cpp
// Compute average spacing using neighborhood of 6 points
double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag> (points, 6);

// Simplify using a grid of size 2 * average spacing
typename Point_set::iterator gsim_it = [CGAL::grid_simplify_point_set](points, 2. * spacing);
points.remove(gsim_it, points.end());
std::cout << points.number_of_removed_points() << " point(s) removed after simplification." << std::endl;

points.collect_garbage();
```

- 平滑
虽然通过“Poisson”或“Scale space”进行的重建可以在内部处理噪声，但有些人可能希望更严格地控制平滑步骤。例如，略微带有噪声的点云可以受益于一些可靠的平滑算法，并通过提供相关属性（具有边界的定向网格）来进行“前进型”的重建。

提供了两个函数来使用良好逼近（即不会降低曲率等）对嘈杂点云进行平滑：
- [jet_smooth_point_set()](https://doc.cgal.org/latest/Point_set_processing_3/group__PkgPointSetProcessing3Algorithms.html#ga96a3738be3b2b9bd1587af78ae10e67a)
- [bilateral_smooth_point_set()](https://doc.cgal.org/latest/Point_set_processing_3/group__PkgPointSetProcessing3Algorithms.html#ga01192a227578fee0c5676ba6a5e88931)

- 法向量估计和定位
Poisson表面重建需要具有定向法线向量的点。要将算法应用于原始点云，必须首先估计法线，例如使用以下两个函数之一：
- [pca_estimate_normals()](https://doc.cgal.org/latest/Point_set_processing_3/group__PkgPointSetProcessing3Algorithms.html#ga721aeb7af4b2d31e08e75bc5d53303cf)
- [jet_estimate_normals()](https://doc.cgal.org/latest/Point_set_processing_3/group__PkgPointSetProcessing3Algorithms.html#ga078e25209373cab0e1f3524202489771)
在高曲率存在的情况下，Jet比PCA更准确。这些函数仅估计法线方向，而不是它们的方向（矢量的方向可能不是局部一致的）。为了正确定位法线，可以使用以下函数：
- [mst_orient_normals()](https://doc.cgal.org/latest/Point_set_processing_3/group__PkgPointSetProcessing3Algorithms.html#ga17c3c558c3799b65bd23fba971a82a5c)
- [scanline_orient_normals()](https://doc.cgal.org/latest/Point_set_processing_3/group__PkgPointSetProcessing3Algorithms.html#gafc3627234666e1fb458e9387e9d5f3c6)
