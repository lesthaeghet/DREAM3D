Convert Data (Voxel Cell) {#convertdata}
=============

## Group (Subgroup) ##
Core Filters (Misc)


## Description ##

This filter converts data from one primitive type to another by using the built in translation of the compiler. This filter can be used if the user needs to convert an array into a type that is accepted by another filter. For example a filter may need an input array to be of type _int32_t_ but the array that the user would like to use is _uint16_t_. The user may use this filter to create a new array that has the proper target type (_int32_t_).

**This filter is here for convenience and should be used with great care and understanding of the input and output data. This filter should rarely be required and if the user thinks that they require this filter then a detailed examination of all the data involved should be undertaken to avoid possible undefined behaviors.**

### IMPORTANT NOTES ###

**Up Casting**

Upcasting is most likely well defined by the compilers. This is the act of creating a new array using a primitive value that is represented by more bytes than the original data. For example if the user converts 1 byte integers into 4 byte integers or converted 4 byte floats into 8 byte floats.

**Down Casting**

Down casting can have undefined behavior depending on the primitive types involved. Down casting is the opposite of up casting and involves converting data from a larger byte count representation to a representation of lower byte count. For example converting 4 byte integers into 2 byte integers or 8 byte floats into 4 byte floats. What happens to the data all depends on the range of values in the original array. If the target type's range can hold all the values of the original arrays values then the conversion would have a well defined outcome.

**Signed/Unsigned Conversions**

When converting data from signed values to unsigned values or vice-versa then there can also be undefined behavior. For example, if the user were to convert a signed 4 byte integer array to an unsigned 4 byte integer array and the input array has negative values then the conversion rules are undefined and may differ from operating system to operating system.

## Parameters ##
| Name             | Type | Description |
|------------------|------|--------------|
| Scalar Type      | Enumeration | Convert to this data type |

## Required Geometry ##
Not Applicable

## Required Arrays ##

| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Any  | Attribute Array Name | Any | (1) | Array to convert |

## Created Arrays ##

| Type | Default Name | Type | Component Dimensions | Description |
|------|--------------|-------------|---------|-----|
| Variable | Output Array Name | Variable | (1) | The converted array |

## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


