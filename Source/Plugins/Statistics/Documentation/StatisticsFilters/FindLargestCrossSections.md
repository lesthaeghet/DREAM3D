Find Feature Largest Cross-Sections {#findlargestcrosssections}
=============

## Group (Subgroup) ##
Statistics Filters (Morphological)

## Description ##
This Filter calculates the largest cross-sectional area on a user-defined plane for all **Features**.  The Filter simply iterates through all **Cells** (on each section) asking for **Feature** that owns them.  On each section, the count of **Cells** for each **Feature** is then converted to an area and stored as the *LargestCrossSection* if the area for the current section is larger than the existing *LargestCrossSection* for that **Feature**.

## Parameters ##
| Name | Type | Description |
|------|------| ----------- |
| Plane of Interest | Choice | Specifies which plane to consider when determining the maximum cross-section for each **Feature** |

## Required Geometry ##
Image / Rectilinear Grid

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | FeatureIds | Int | (1) | Specifies to which **Feature** each **Cell** belongs. |

## Created Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Feature | LargestCrossSection | Float | (1) | Area of largest cross-section for **Feature** perpendicular to the user specified direction. |


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


