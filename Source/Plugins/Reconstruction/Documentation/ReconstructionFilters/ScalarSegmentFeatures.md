Segment Features (Scalar) {#scalarsegmentfeatures}
======

## Group (Subgroup) ##
Reconstruction Filters (Segmentation)

## Description ##
This Filter segments the **Features** by grouping neighboring **Cells** that satisfy the *scalar tolerance*, i.e., have a scalar difference less than the value set by the user. The process by which the **Features** are identified is given below and is a standard *burn algorithm*.

1) Randomly select a **Cell**, add it to an empty list and set its *FeatureId* to the current **Feature**.
2) Compare the **Cell** to each of its six (6) face-sharing neighbors (ie calculate the scalar difference with each neighbor)
3) Add each neighbor **Cell** that has a scalar difference below the user defined tolerance to the list created in 1) and set the *FeatureId* of the neighbor **Cell** to the current **Feature**.
4) Remove the current **Cell** from the list and move to the next **Cell** and repeat 2) and 3) - continue until no **Cells** are left in the list
5) Increment the current **Feature** counter and repeat steps 1) through 4) - continue until no **Cells** remain unassigned in the dataset.

The user has the option to *Use Mask Array*, which allows the user to set a boolean array for the **Cells** that remove **Cells** with a value of *false* from consideration in the above algorithm. This option is useful if the user has an array that either specifies the domain of the "sample" in the "image" or specifies if the orientation on the **Cell** is trusted/correct. 

After all the **Features** have been identified, an **Attribute Matrix** is created for the **Features** and each **Feature** is flagged as *Active* in a boolean array in the matrix.

## Parameters ##
| Name | Type | Description |
|------|------| ----------- |
| Scalar Tolerance | Float | Tolerance  used to determine if neighboring **Cells** belong to the same **Feature** |
| Use Mask Array | Boolean | Specifies whether to use a boolean array to exclude some **Cells** from the **Feature** identification process |

## Required Geometry ##
Image / Rectilinear Grid

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | None | Varies | (1) | Scalar array used during segmentation |
| Cell | GoodVoxels | Boolean | (1) | Specifies if the **Cell** is to be considered during segmentation.  Array only required if *Use Mask Array* is set to *true* |

## Created Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | FeatureIds | Int | (1) | Specifies to which **Feature** each **Cell** belongs. |
| Feature | Active | Boolean | (1) | Specifies if the **Feature** is still in the sample (1 if the **Feature** is in the sample and 0 if it is not). At the end of the filter, all **Features** will be "Active". |


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


