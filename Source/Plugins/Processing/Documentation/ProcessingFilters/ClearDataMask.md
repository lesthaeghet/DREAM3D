Clear Data (Mask) {#cleardatamask}
=============

## Group (Subgroup) ##
Processing Filters (Cleanup)

## Description ##
This Filter clears all attribute arrays for each **Element** that have a value of *false* in the _mask_ array.  All **Elements** that have the value of *true* in the _mask_ array will retain all their attributes.

## Parameters ##
None

## Required Geometry ##
Not Applicable

## Required Arrays ##
| Type | Default Name | Type | Component Dimensionss | Description |
|------|--------------|-------------|---------|-----|
| Element | GoodVoxels | Boolean | (1) | Used to define whether the **Elements** are part of the mask  |

## Created Arrays ##
None


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users



