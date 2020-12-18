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
// Authors: Asher Elmquist
// =============================================================================
//
// utility functions used for optix convenience
//
// =============================================================================

#include "chrono_sensor/optixcpp/ChOptixUtils.h"
#include <cstring>
#include <fstream>
#include <sstream>

namespace chrono {
namespace sensor {

using namespace optix;

CH_SENSOR_API std::string ptxFromFile(std::string file_name) {
    std::string str;
    std::ifstream f(file_name.c_str());
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        str = source_buffer.str();
        return str;
    }
    return "";
}

CH_SENSOR_API optix::Program GetRTProgram(optix::Context context, std::string file_name, std::string program_name) {
    std::string ptx_file = std::string(PTX_GENERATED_PATH) + ptx_pre + file_name + ptx_suff;
    std::string ptx = ptxFromFile(ptx_file);

    optix::Program program = context->createProgramFromPTXString(ptx, program_name);
    return program;
}

ByteImageData LoadImage(std::string filename) {
    ByteImageData img_data;
    int w;
    int h;
    int c;
    unsigned char* data = stbi_load(filename.c_str(), &w, &h, &c, 0);

    if (!data) {
        img_data.w = 0;
        img_data.h = 0;
        img_data.c = 0;
        return img_data;  // return if loading failed
    }

    img_data.data = std::vector<unsigned char>(w * h * c);
    img_data.w = w;
    img_data.h = h;
    img_data.c = c;
    memcpy(img_data.data.data(), data, sizeof(unsigned char) * img_data.data.size());

    // for (int i = 0; i < h; i++) {
    //     for (int j = 0; j < w; j++) {
    //         for (int k = 0; k < c; k++) {
    //             png_data.data[i * w * c + j * c + k] = data[i * w * c + j * c + k];
    //         }
    //     }
    // }
    stbi_image_free(data);

    return img_data;
}

}  // namespace sensor
}  // namespace chrono
