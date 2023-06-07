
https://pcl.readthedocs.io/projects/tutorials/en/master/normal_estimation.html#normal-estimation

The solution for estimating the surface normal is therefore reduced to an analysis of the eigenvectors and eigenvalues (or PCA – Principal Component Analysis) of a covariance matrix created from the nearest neighbors of the query point. More specifically, for each point ${p}_i$, we assemble the covariance matrix $\mathcal{C}$ as follows:

$$
\mathcal{C} = \frac{1}{k}\sum_{i=1}^{k}{\cdot (\boldsymbol{p}_i-\overline{\boldsymbol{p}})\cdot(\boldsymbol{p}_i-\overline{\boldsymbol{p}})^{T}}, ~\mathcal{C} \cdot \vec{{\mathsf v}_j} = \lambda_j \cdot \vec{{\mathsf v}_j},~ j \in \{0, 1, 2\}
$$

Where $k$ is the number of point neighbors considered _in the neighborhood of ${p}_i$, $\overline{\boldsymbol{p}}$ represents the 3D centroid of the nearest neighbors, $\lambda_j$ is the j-th eigenvalue of the covariance matrix, and $\vec{{\mathsf v}_j}$ the j-th eigenvector.

To estimate a covariance matrix from a set of points in PCL, you can use:

```cpp
Eigen::Matrix3f covariance_matrix;
Eigen::Vector4f xyz_centroid;

//Estimate the XYZ centroid
compute3DCentroid(cloud, xyz_centroid);

//Compute the 3x3 covariance matrix
computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);

```

In general, because there is no mathematical way to solve for the sign of the normal, its orientation computed via Principal Component Analysis (PCA) as shown above is ambiguous, and not consistently oriented over an entire point cloud dataset. The figure below presents these effects on two sections of a larger dataset representing a part of a kitchen environment. The right part of the figure presents the Extended Gaussian Image (EGI), also known as the normal sphere, which describes the orientation of all normals from the point cloud. Since the datasets are 2.5D and have thus been acquired from a single viewpoint, normals should be present only on half of the sphere in the EGI. However, due to the orientation inconsistency, they are spread across the entire sphere.

![[Pasted image 20230605143040.png|500]]

The solution to this problem is trivial if the viewpoint $v_p$ is in fact known. To orient all normals $\vec{\boldsymbol{n}}_i$ consistently towards the viewpoint, they need to satisfy the equation:
$$
\vec{\boldsymbol{n}}_i \cdot ({\mathsf v}_p - \boldsymbol{p}_i) > 0
$$

![[Pasted image 20230605143407.png|500]]

