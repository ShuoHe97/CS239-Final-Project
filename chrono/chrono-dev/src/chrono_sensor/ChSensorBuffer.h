// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHSENSORBUFFER_H
#define CHSENSORBUFFER_H
// #pragma once

#include <optix.h>
#include <optixu/optixpp.h>  //needed to make sure things are in the right namespace. Must be done before optixpp_namespace.h
#include <optixu/optixpp_namespace.h>  //is covered by optixpp.h but will be removed from optixpp.h in the future
#include <functional>
#include <memory>

namespace chrono {
namespace sensor {

struct SensorBuffer {
    SensorBuffer() : Width(0), Height(0), UpdateCount(0) {}
    SensorBuffer(unsigned int w, unsigned int h) : Width(w), Height(h), UpdateCount(0) {}
    virtual ~SensorBuffer() {
    }  // virtual destructor so class is virtual so it can participate in dynamic_pointer_cast<>'s
    unsigned int Width;
    unsigned int Height;
    unsigned int
        UpdateCount;  // how many times we've updated this buffer (way to allow users to check for buffer updates)
};

// Base class of 2D buffers
// (Do not use this class directly, instead use the typedefs below)
template <class B>
struct SensorBufferT : public SensorBuffer {
    SensorBufferT() {}
    B Buffer;
};

//================================
// RGBA8 Pixel Format and Buffers
//================================

// a pixel in RGBA 8bpp format
struct PixelRGBA8 {
    char R;
    char G;
    char B;
    char A;
};

//============================================================================
// Buffer of Optix memory (contents described by members inside optix::Buffer)
//============================================================================

using SensorOptixBuffer = SensorBufferT<optix::Buffer>;

//===================================================
// RGBA8 (32bpp RGB + Alpha) Pixel Format and Buffers
//===================================================

// typedef for a RGBA8 Image Buffer Stored in CPU memory
using SensorHostRGBA8Buffer = SensorBufferT<std::unique_ptr<PixelRGBA8[]>>;

// typedefs for a RGBA8 Buffer Stored in GPU memory
using DeviceRGBA8BufferPtr = std::unique_ptr<PixelRGBA8[], std::function<void(PixelRGBA8*)>>;
using SensorDeviceRGBA8Buffer = SensorBufferT<DeviceRGBA8BufferPtr>;

// typedef for a unique pointer to a Sensor Host (CPU) RGBA8 Buffer with custom deleter
using LockedRGBA8BufferPtr = std::unique_ptr<SensorHostRGBA8Buffer, std::function<void(SensorHostRGBA8Buffer*)>>;

//===============================================
// R8 (8-bit Grayscale) Pixel Format and Buffers
//===============================================

// typedef for a R8 (Graycale) Image Buffer Stored in CPU memory
using SensorHostR8Buffer = SensorBufferT<std::unique_ptr<char[]>>;

// typedefs for a R8 (Grayscale) Buffer Stored in GPU memory
using DeviceR8BufferPtr = std::unique_ptr<char[], std::function<void(char*)>>;
using SensorDeviceR8Buffer = SensorBufferT<DeviceR8BufferPtr>;

// typedef for a unique pointer to a Sensor Host(GPU) 8-bit grayscale Buffer with custom deleter
using LockedR8BufferPtr = std::unique_ptr<SensorHostR8Buffer, std::function<void(SensorHostR8Buffer*)>>;

//===============================
// Lidar Data Format and Buffers
//===============================

// Lidar data in generic format
struct PixelDI {
    float range;
    float intensity;
};

struct PixelXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

// typedef for a IMU Data Buffer Stored in CPU memory
using SensorHostXYZIBuffer = SensorBufferT<std::unique_ptr<PixelXYZI[]>>;
using SensorHostDIBuffer = SensorBufferT<std::unique_ptr<PixelDI[]>>;

using DeviceXYZIBufferPtr = std::unique_ptr<PixelXYZI[], std::function<void(PixelXYZI*)>>;
using SensorDeviceXYZIBuffer = SensorBufferT<DeviceXYZIBufferPtr>;

using DeviceDIBufferPtr = std::unique_ptr<PixelDI[], std::function<void(PixelDI*)>>;
using SensorDeviceDIBuffer = SensorBufferT<DeviceDIBufferPtr>;

// typedef for a unique pointer to a Sensor Host (CPU) lidar Data Buffer with custom deleter
using LockedXYZIBufferPtr = std::unique_ptr<SensorHostXYZIBuffer, std::function<void(SensorHostXYZIBuffer*)>>;
using LockedDIBufferPtr = std::unique_ptr<SensorHostDIBuffer, std::function<void(SensorHostDIBuffer*)>>;

//======================
// Depth camera buffers
//======================

// Lidar data in generic format

struct PixelXYZRGB {
    float x;
    float y;
    float z;
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char none;
};

// typedef for a IMU Data Buffer Stored in CPU memory
using SensorHostXYZRGBBuffer = SensorBufferT<std::unique_ptr<PixelXYZRGB[]>>;

using DeviceXYZRGBBufferPtr = std::unique_ptr<PixelXYZRGB[], std::function<void(PixelXYZRGB*)>>;
using SensorDeviceXYZRGBBuffer = SensorBufferT<DeviceXYZRGBBufferPtr>;

// typedef for a unique pointer to a Sensor Host (CPU) IMU Data Buffer with custom deleter
using LockedXYZRGBBufferPtr = std::unique_ptr<SensorHostXYZRGBBuffer, std::function<void(SensorHostXYZRGBBuffer*)>>;
// using LockedSensorDeviceXYZRGBBufferPtr =
//     std::unique_ptr<SensorDeviceXYZRGBBuffer, std::function<void(SensorDeviceXYZRGBBuffer*)>>;

//=============================
// IMU Data Format and Buffers
//=============================

// IMU data in generic format
struct IMUData {
    double Accel[3];
    double Roll;
    double Pitch;
    double Yaw;
};

// typedef for a IMU Data Buffer Stored in CPU memory
using SensorHostIMUBuffer = SensorBufferT<std::unique_ptr<IMUData[]>>;

// typedef for a unique pointer to a Sensor Host (CPU) IMU Data Buffer with custom deleter
using LockedIMUBufferPtr = std::unique_ptr<SensorHostIMUBuffer, std::function<void(SensorHostIMUBuffer*)>>;

//============================
// GPS Data Format and Buffers
//============================

// GPS data in generic format
struct GPSData {
    double Latitude;
    double Longitude;
    double Altitude;
    double Time;
};

// typedef for a GPS Data Buffer Stored in CPU memory
using SensorHostGPSBuffer = SensorBufferT<std::unique_ptr<GPSData[]>>;

// typedef for a unique pointer to a Sensor Host (CPU) GPS Data Buffer with custom deleter
using LockedGPSBufferPtr = std::unique_ptr<SensorHostGPSBuffer, std::function<void(SensorHostGPSBuffer*)>>;

}  // namespace sensor
}  // namespace chrono

#endif
