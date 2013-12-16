# Introduction #

In order to setup QtCreator on Windows you need a few items

## Preliminaries ##

+ [QtCreator 2.8.x installed](http://qt-project.org/downloads)
+ Visual Studio Installed
+ DREAM3D_SDK installed int C:\DREAM3D_SDK_X64

1. Launch QtCreator.
2. File->Open File or Project
3 Select the DREAM3D/CMakeLists.txt file
4. Select the CMake.exe program located on your computer
5. Click "Next"
6. Select the Generator for your system (NMake Generator) unless you have Ninja installed.
7. Paste the following into the "Arguments" line

    -DHDF5_INSTALL=C:\Developer\DREAM3D_SDK_X64\hdf5-1.8.10.1 -DEIGEN_INSTALL=C:\Developer\DREAM3D_SDK_X64\Eigen-3.1.2 -DTBB_INSTALL_DIR=C:\Developer\DREAM3D_SDK_X64\tbb41_20130116oss -DQWT_INSTALL=C:\Developer\DREAM3D_SDK_X64\Qwt-5.2.1 -DBOOST_ROOT=C:\Developer\DREAM3D_SDK_X64\MXABoost-1.44 -DDOXYGEN_EXECUTABLE=C:\Developer\DREAM3D_SDK_X64\doxygen\bin\doxygen.exe -DDREAM3D_DATA_DIR=../../DREAM3D_DATA

8. Click the "Run CMake" button and let it configure.
9. Assuming there are no CMake errors, click the "Finish" button.
    
When the project loads pressing "ctrl-b" will build the project. The debugging is slower than in Visual Studio but the editor is better in our opinion.

