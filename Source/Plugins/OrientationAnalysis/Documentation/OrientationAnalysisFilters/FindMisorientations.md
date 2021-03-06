Find Feature Neighbor Misorientations {#findmisorientations}
==========

## Group (Subgroup) ##
Orientation Analysis Filters (Statistics)

## Description ##
This filter determines, for each **Feature**, the misorientations with each of the **Features** that are in contact with it.  The misorientations are stored as a list (for each **Feature**) of angles (in degrees).  The axis of the misorientation is not stored by this filter.

## Parameters ##
| Name | Type | Description |
|------|------|------|
| Find Average Misorientation Per Feature | Boolean | Specifies if the *average* of  the misorienations with the neighboring **Features** should be stored for each **Feature**. |

## Required Geometry ##
Not Applicable

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Feature | NeighborLists | List of Ints | (1) | List of the contiguous neighboring **Features** for a given **Feature** |
| Feature | AvgQuats | Float | (4) | Defines the average orientation of the **Feature** in quaternion representation  (<x,y,z>, w) |
| Feature | FeaturePhases | Int | (1) | Specifies the phase of the **Feature** - Values will begin at 1 as there is no phase 0, which is used temporarily in some filters for bad data|
| Ensemble | CrystalStructures | Int | (1) | Specifies the crystal structure of each Ensemble using an enumeration defined by DREAM3D (Hexagonal_High=0, Cubic_High=1, Hexagonal_Low=2, Cubic_Low=3, Triclinic=4, Monoclinic=5, Orthorhombic=6, Tetragonal_Low=7, Tetragonal_High=8, Trigonal_Low=9, Trigonal_High=10, Unknown=999) |

## Created Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Feature | MisorientationLists | List of Floats | (1) | List of the misorientation angles with the contiguous neighboring **Features** for a given **Feature** |
| Feature | AvgMisorientation | Float | (1) | Number weighted average of neighbor misorientations. Only created if Find Average Misorientation Per Feature is checked |


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


