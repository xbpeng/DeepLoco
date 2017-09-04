# Intro

This project is designed to learn good navigation skills for simulated characters


# Setup

This section covers some of the steps to setup and compile the code. The software depends on many libraries that need to be carefully prepared and placed for the building and linking to work properly.

## Linux (Ubuntu 16.04)

### Install system dependencies

Run the `deb_deps.sh` script to install the system dependencies required to build the project.


### OpenGL >= 3.3

Ensure that your machine is capable of running OpenGL version 3.3 or greater.

You can verify the version of OpenGL your machine currently has installed via the following command:
```
glxinfo | grep "OpenGL version"
```

If `glxinfo` is not installed on your system, install the `mesa-utils` package.

OpenGL should come as part of the drivers for your graphics hardware (whether part of the motherboard or dedicated graphics card). If you are missing a compatible version of OpenGL, consider updating your graphics drivers; if you have a GPU, ensure that the system is actually using it.


### Download premake4
Download premake4 from [here](https://sourceforge.net/projects/premake/files/Premake/4.4/premake-4.4-beta5-linux.tar.gz/download). Extract the premake4 binary into a directory of your choosing (preferably not your local Downloads directory).

Add premake to your `PATH` variable in `.bashrc`:
```
# Add premake to path
export PATH=[PREMAKE_DIR]:$PATH
```
...where [PREMAKE_DIR] should be the directory containing the premake4 binary.


### Build Instructions

1. Download the most recent compressed external file from the newest release. 
1. Extract it and move into the DeepLoco directory. The top directory of the DeepLoco repository should now contain a directory called `external`, in addition to all the other directories that were there before.
1. Build the source code for caffe that came in the `external` directory.
	```
	cd external/caffe
	make clean
	make
	cd ../../
	```
1. Copy the newly-compiled caffe lib directory from external/caffe/build/lib to the top directory of DeepLoco.
	```
	cp -r external/caffe/build/lib .
	```
1. Generate makefiles using premake4.
	```
	premake4 clean
	premake4 gmake
	```
1. Build all the targets!
	```
	cd gmake
	make config=debug64
	```
	Note: you can speed up the build by appending the `-j8` flag to this last `make` command, where `8` here is the number of concurrent build threads that `make` will launch. Choose a number that makes sense based on the hardware resources you have available to you.

1. Copy the prebuilt libraries from the external folder

```
cp external/caffe/build/lib/libcaffe.* lib/
cp external/Bullet/bin/*.so lib/
cp external/jsoncpp/build/debug/src/lib_json/*.so* lib/
``` 
 
**Note:** There are some issues with the installation on Ubuntu 14.04. Some of the libraries have changed their location and name (see https://github.com/BVLC/caffe/issues/2347 for a solution).

## Windows

This setup has been tested on Windows 7 and 10 with visual studio 2013.

  1. Download the library.zip file that contains almost all of the relevant pre compiled external libraries and source code.
  2. Unpack this library in the same directory the project is located in. For example, DeepLoco/../.
  3. You might need to install opengl/glu/GL headers. We have been using freeglut for this project. glew might already be included in library.zip.
  4. You will need to copy some dll files from dynamic_lib.zip to the directory the project is compiled to. For example, optimizer/x64/Debug/. These files are needed by the framework during runtime.
  5. Might need to create a folder in DeepLoco called "output", This is where temprary and current policies will be dumped.

## Running The System

After the system has been build there are two executable files that server different purposes. The **DeepLoco** program is for visually simulating the a controller and **DeepLoco_Optimize** is for optimizing the parameters of some controller.

Examples:  
	To simulate a controller/character  
	./DeepLoco -arg_file= args/test_args.txt
	To simulate a controller/character with a specific policy  
	./DeepLoco_Optimizer -arg_file= args/opt_int_poli_hopper_eval.txt -policy_model= output/intermediate/trainer_int_model_0000160000.h5  
	To Train a controller  
	./DeepLoco_Optimizer -arg_file= args/opt_args_train.txt  
	./DeepLoco_Optimizer -arg_file= args/opt_args_train_hopper.txt  
	To Optimize a controllers parameters  
	./DeepLoco_Optimizer -arg_file= args/opt_args_jump.txt  


## Key Bindings

Most of these are toggles

 - c fixed camera mode
 - y draw COM path and contact locations
 - q draw "filmstrip" like rendering
 - f draw torques
 - h draw Actor value functions?
 - shift + '>' step one frame
 - p toggle draw value function
 - ',' and '.' change render speed, decrease and increase.
 - "spacebar" to pause simulation
 - r restart the scenario
 - l reload the simulation (reparses the arg file)
 - g draw state features

## Contents of external dependencies folders
These lists are provided for reference only. Normally, if you follow the instructions above, you shouldn't need to know about any of this.

### Linux
 - caffe source code (must still be built)
 	- Specific version (https://github.com/niuzhiheng/caffe.git @ 7b3e6f2341fe7374243ee0126f5cad1fa1e44e14)
	 - 	In the instruction to make and build Caffe we uncomment the CPU only line  
		```
		# CPU-only switch (uncomment to build without GPU support).
		CPU_ONLY := 1
		```
 - BulletPhysics ([This specific threadsafe version](https://github.com/lunkhound/bullet3))
 - [Json_cpp](https://github.com/open-source-parsers/jsoncpp)
 - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
 - [CMA-ES](https://github.com/AlexanderFabisch/CMA-ESpp)  
 - [LodePNG](https://github.com/lvandeve/lodepng)

### Windows
 - Caffe: https://github.com/initialneil/caffe-vs2013
 - TODO: Finish documenting this
