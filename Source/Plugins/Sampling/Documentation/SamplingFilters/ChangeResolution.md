Change Resolution {#changeresolution}
=============

## Group (Subgroup) ##
Sampling Filters (Resolution)

## Description ##
This Filter changes the **Cell** spacing/resolution based on inputs from the user.  The values entered are the desired new resolutions (not multiples of the current resolution).  The number of **Cells** in the volume will likely change when the resolution values are changed and thus the user should be cautious of generating "too many" **Cells** by entering very small values (i.e. very high resolution).  

A new grid of **Cells** is created and "overlaid" on the existing grid of **Cells**.  There is currently no *interpolation* performed, rather the attributes of the old **Cell** that is closest to each new **Cell**'s is assigned to that new **Cell**.

## Parameters ##
| Name | Type | Description |
|------|------|------|
| Resolution | Double (x3) | Vector of new resolution values (dx, dy, dz) |
| Save as New DataContainer | Boolean | Specifies if the new grid of **Cells** should replace the current geometry or if a new **Data Container** should be created to hold it |
| Renumber Features | Boolean | Specifies if the **Features** should be renumbered if some **Features** 'disappear' when changing the resolution (ie coarsening the resolution may cause small **Features** to be removed. |

## Required Geometry ##
Image / Rectilinear Grid

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | FeatureIds | Int | (1) | Specifies to which **Feature** each **Cell** belongs. (Only required if Renumber Features is *true* |

## Created Arrays ##
None

## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


