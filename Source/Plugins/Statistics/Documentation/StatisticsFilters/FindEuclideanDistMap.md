Find Euclidean Distance Map {#findeuclideandistmap}
=============

## Group (Subgroup) ##
Statistics Filters (Morphological)

## Description ##
This Filter calculates the distance of each **Cell** from the nearest **Feature** boundary, triple line and/or quadruple point.  The following algorithm explains the process:

1. Find the **Feature** that owns each **Cell** and its six face-face neighbors of each **Cell**
2. For all **Cells** that have *at least 2* different neighbors, set their *GBEuclideanDistance* to *0*.  For all **Cells** that have *at least 3* different neighbors, set their *TJEuclideanDistance* to *0*.  For all **Cells** that have *at least 4* different neighbors, set their *QPEuclideanDistance* to *0*.
3. For each of the three *EuclideanDistace* maps, iteratively "grow" out from the **Cells** identified to have a distance of *0* by the following sub-steps:

  - Determine the **Cells** that neighbor a **Cell** of distance *0* in the current map.
  - Assign a distance of *1* to those **Cells** and list the *0* **Cell** neighbor as their *nearest neighbor*
  - Repeat previous two sub-steps, increasing the distances by *1* each iteration, until no **Cells** remain without a distance and *nearest neighbor* assigned.

	*Note:* the distances calculated at this point are "city-block" distances and not "shortest distance" distances.

4. If the option *Calculate Manhattan Distance Only* is *false*, then the "city-block" distances are overwritten with the *Euclidean Distance* from the **Cell** to its *nearest neighbor* **Cell**.


## Parameters ##
| Name | Type | Description |
|------|------| ----------- |
| Calculate Manhattan Distance Only | Boolean | Specifies whether the distance to boundaries, triple lines and quadruple points is stored as "city block" or "Euclidean" distances |
| Calculate Distance to Boundaries | Boolean | Specifies if the distance of each **Cell** to a **Feature** boundary is calculated |
| Calculate Distance to Triple Lines | Boolean | Specifies if the distance of each **Cell** to a triple line between **Features** is calculated |
| Calculate Distance to Quadruple Points | Boolean | Specifies if the distance of each **Cell** to a  quadruple point between **Features** is calculated |
| Store the Nearest Boundary Cells | Boolean | Specifies whether to store the *nearest neighbors* of **Cell**  


## Required Geometry ##
Image / Rectilinear Grid

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | FeatureIds | Int | (1) | Specifies to which **Feature** each **Cell** belongs. |

## Created Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | GBEuclideanDistances | Float | (1) | Distance the **Cells** are from the *boundary* of the **Feature** they belong to. |
| Cell | TJEuclideanDistances | Float | (1) | Distance the **Cells** are from a *triple junction* of **Features** . |
| Cell | QPEuclideanDistances | Float | (1) | Distance the **Cells** are from a *quadruple point* of **Features** . |
| Cell | NearestNeighbors | Int  | (3) | Indices of the closest **Cell** that touches a boundary, triple and quadruple point for each **Cell** |


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


