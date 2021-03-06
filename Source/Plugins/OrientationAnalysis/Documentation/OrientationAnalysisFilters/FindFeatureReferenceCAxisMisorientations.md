Find Feature Reference C-Axis Misalignments {#findfeaturereferencecaxismisorientations}
======

## Group (Subgroup) ##
Orientation Analysis Filters (Statistics)

## Description ##
This filter calculates the misorientation angle between the c-axis of each **Cell** within a **Feature** and the average c-axis for that **Feature** and stores that value for each **Cell**.  The average and standard deviation of those values for all **Cells** belonging to the same **Feature** is also stored for each **Feature**.

## Parameters ##
None

## Required Geometry ##
Not Applicable

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Element | FeatureIds | Int | (1) | Specifies to which **Feature** each **Cell** belongs. |
| Element | Quats | Float | (4) | Specifies the orientation of the **Element** in quaternion representation |
| Element     | Phases            | Int | (1) | Specifies the **Ensemble** of the **Cell** |
| Feature | AvgCAxes | Float | (3) | The direction <u,v,w> of the **Feature's** c-axis in the sample reference frame |

## Created Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Element | FeatureReferenceCAxisMisorientations | Float | (1) | Misorientation angle (in degrees) between **Element**'s c-axis and the c-axis of the **Feature** that owns that **Element** |
| Feature | FeatureAvgCAxisMisorientations | Float | (1) | Average of the *FeatureReferenceCAxisMisorientation* values for all of the **Elements** that belong to the **Feature** |
| Feature | FeatureStdevCAxisMisorientations | Float | (1) | Standard deviation of the *FeatureReferenceCAxisMisorientation* values for all of the **Elements** that belong to the **Feature** |


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


