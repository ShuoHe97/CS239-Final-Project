# Chrono Sensor Module
Tested Systems:
 - Arch Linux: 7/30/2019
 - Windows 10: 7/28/2019

## Supported Sensor
 - RGB Mono Camera

## Dependencies
 - NVIDIA GPU
	 - tested on Maxwell and later
	 - should work with Kepler and later - restricted by OptiX and Cuda
 - OptiX (required)
	 - 6.0.0
 - Cuda (required)
	 - 10.1 (tested on Linux and Windows)
	 - 10.0 (untested)
 - GLFW >= 3.? (required)
 - GLEW >= ?? (required)
 - openGL >= ?? (required)
 - TODO: add TensorRT (optional)

## CMake and Build Notes
 - Recommended to build and NOT install. Use the build directory rather than installing to a system directory
 - OptiX cmake paths that must be set
	 - liboptix.so
	 - liboptixu.so
	 - liboptix_prime.so
	 - optix include path: path to included directory that contains files such as optix.h

## Getting started with the demos
 - demo_SEN_buildtest
	 - builds if Chrono_Sensor and Build_Demos are enabled
	 - includes falling objects and camera sensor. A filter graph is added to the camera(s) that displays the original render, converts to greyscale, then displays greyscale image
 - demo_SEN_HMWMV
	 - only builds if Chrono_Sensor, Build_Demos, Chrono_Vehicle, and Chrono_Irrlicht are enabled
	 - runs a chrono vehicle with multiple sensor attached to the vehicle. This is the starting point for a typical autonomous vehicle simulation.
 - demo_SEN_sedan
	 - sedan demo for running a simulation along Park St. in Madison, WI. Mesh from Asher Elmquist is needed to run in simulation properly



## Current Capabilities
 - Scene Rendering
	 - lights
		 - basic point light
		 - shadows
	 - Materials
		 - reflection based on material reflectance
		 - fresnel effect
		 - mesh support based on Wavefront OBJ format
		 - programmatic material creation
		 - partial transparency without refractance
	 - Objects
		 - Box primitives
		 - Sphere primitives
		 - Triangle Mesh
 - Camera sensor
	 - ground-truth ray-traced camera rendering
	 - filter-based sensor model for user defined sensor model
 - Filters
	 - Greyscale kernel
	 - visualization using GLFW
	 - copy-back filter for data access from CPU
	 - save images to file at a specific path

## Capabilities in Progress
 -
