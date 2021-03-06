Reverse Triangle Winding Filter {#reversetrianglewinding}
======

## Group (Subgroup) ##
SurfaceMesh

## Description ##
This filter reverses the winding on each triangle. If the next filter is to compute the surface normals the user will notice that the normals now point in the opposite direction. Some analysis filters require the normals to be pointing "away" from the center of a **Feature**. The current surface meshing algorithms tend to orient the normals pointing "into" the **Feature**.


## Parameters ##

## Required DataContainers ##
SurfaceMesh - Valid Surface Mesh containing the shared vertex array and face list

## Required Arrays ##
None


## Created Arrays ##
None


## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


