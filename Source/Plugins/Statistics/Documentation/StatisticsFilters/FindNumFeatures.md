Find Number of Features {#findnumfeatures}
=============

## Group (Subgroup) ##
Statistics Filters (Morphological)

## Description ##
This Filter determines the number of **Features** in each **Ensemble**.

## Parameters ##
None 

## Required Geometry ##
Not Applicable

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Feature | Phases | Int | (1) | Specifies the **Ensemble** of the **Feature** - Values will begin at 1 as there is no **Ensemble** 0, which is used temporarily in some filters for bad data|

## Created Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Ensemble | NumFeatures | Int | (1) | Number of **Features** that belong each **Ensemble**. |

## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


