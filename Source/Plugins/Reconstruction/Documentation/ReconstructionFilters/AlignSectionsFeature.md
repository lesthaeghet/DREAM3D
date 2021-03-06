Align Sections (Feature) {#alignsectionsfeature}
======

## Group (Subgroup) ##
Reconstruction Filters (Alignment)

## Description ##
This Filter attempts to align 'sections' of the sample perpendicular to the z-direction by determining the position that results in the most overlap of previously defined "regions".  The "regions" are defined by a binary/boolean array where the **Cells** have been flagged by another Filter.  Typically, during reading/processing of the data, each **Cell** is subject to a "Quality Metric" (or threshold) that defines if the **Cell** is "good".  This threshold can be used to define areas of each slice that are bad, either due to actual **Features** in the microstructure or external references inserted by the user/experimentalist.  If these "regions" of "bad" **Cells** are believed to be consistent through sections, then this filter will preserve that by aligning those "regions" on top of one another on consecutive sections. The algorithm of this filter is as follows:

1. Compare the value of the binary/boolean array for each **Cell** in a section with the value of the array for the **Cell**  directly above it in the next section.  
2. Count the number of **Cell** pairs that do not have the same value and store that as the misalignment value for that position.
3. Repeat steps 1 and 2 for each position when shifting the second slice (relative to the first) from three (3) **Cells** to the left to three (3) **Cells** to the right, as well as from three (3) **Cells** up to three (3) **Cells** down. *Note that this creates a 7x7 grid*
4. Determine the position in the 7x7 grid that has the lowest misalignment value. (It will be the position with the fewest different **Cell** pairs).
5. Repeat steps 1-4 with the center of each (new) 7x7 grid at the best position from the last 7x7 grid until the best position in the current/new 7x7 grid is the same as the last 7x7 grid. 
6) Repeat steps 1-5 for each pair of neighboring sections. 

*Note that this is similar to a downhill simplex and can get caught in a local minima*
 
The user can choose to write the determined shift to an output file by enabling *Write Alignment Shifts File* and providing a file path.  

The user can also decide to remove a 'background shift' present in the sample. The process for this is to fit a line to the x and y shifts along the z-direction of the sample.  The individual shifts are then modified to make the slope of the fit line be 0.  Effectively, this process is trying to keep the top and bottom section of the sample fixed.  Some combinations of sample geometry and internal features can result in this algorithm introducing a 'shear' in the sample and the *Linear Background Subtraction* will attempt to correct for this.

## Parameters ##
| Name | Type | Description |
|------|------| ----------- |
| Write Alignment Shift File | Boolean | Specifies whether to write the shifts applied to each section to a file |
| Alignment File | File Path | Path where the user would like the shifts applied to the section to be written. (Only required if *Write Alignment Shifts File* is set to *true* |
| Linear Background Subtraction | Boolean | Specifies if the user wants to remove a 'background' shift present in the alignment |

## Required Geometry ##
Image / Rectilinear Grid

## Required Arrays ##
| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Cell | GoodVoxels | Boolean | (1) | Specifies if the **Cell** is to be counted as part of the *Sample* or not |

## Created Arrays ##
None

## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


