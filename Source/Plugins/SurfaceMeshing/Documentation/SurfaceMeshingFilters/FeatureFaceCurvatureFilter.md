Feature Face Curvature Filter {#featurefacecurvaturefilter}
======

## Group (Subgroup) ##
SurfaceMesh

## Description ##
This filter calculates the mean and Gaussian curvature for each triangle in a surface mesh using the technique in [1]
and where the groups of triangles are defined as those triangles having the same pair of **Feature** ids (nSpin values).

The Principal Curvature 1 &amp; 2 are the "Kappa 1" and "Kappa 2" values from the paper and are the Eigen values from the Wiengarten Matrix.

The Principal Direction 1 &amp; 2 are the Eigen vectors from the solution to the least squares fit algorithm.

The Mean Curvature is (Kappa1 + Kappa2)/2

The Gaussian Curvature is (Kappa1 * Kappa2).
__[1]__
Jack Goldfeather, Victoria Interrante "A Novel Cubic-Order Algorithm for Approximating Principal Direction Vectors"
_ACM Transactions on Graphics_ 2004, 23(1), pp. 45-63.

![Sample Output from filter](FeatureFaceCurvatureFilter.png)
@image latex FeatureFaceCurvatureFilter.png " " width=6in


## Parameters ##

| Name | Type | Description |
|------|------| ----------- |
| Neighborhood Ring Count | Integer | The size of the nieghborhood to use to calculate the curvature values |
| Compute Principal Directions | Bool | Compute the Principal Direction Vectors |
| Compute Mean Curvature | Bool | Compute the Mean Curvature values |
| Compute Gaussian Curvature | Bool | Compute the Gaussian Curvature values |

## Required DataContainers ##
SurfaceMesh - Valid Surface Mesh containing the shared vertex array and face list

## Required Arrays ##
None

## Created Arrays ##

| Type | Default Name | Comment |
|------|--------------|---------|
| Triangle Attribute Array | Principal Curvature 1 | N x 1 Col array of double values |
| Triangle Attribute Array | Principal Curvature 2 | N x 1 Col array of double values |
| Triangle Attribute Array | Principal Direction 1 | N x 3 Col array of double values |
| Triangle Attribute Array | Principal Direction 2 | N x 3 Col array of double values |
| Triangle Attribute Array | Mean Curvature | N x 1 Col array of double values |
| Triangle Attribute Array | Gaussian Curvature | N x 1 Col array of double values |


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


