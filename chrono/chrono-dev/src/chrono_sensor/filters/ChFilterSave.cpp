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
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/ChSensor.h"

#include "chrono_thirdparty/nothings/stb_image_write.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSave::ChFilterSave(std::string data_path) : ChFilter("") {
    m_path = data_path;
}

CH_SENSOR_API ChFilterSave::~ChFilterSave() {}

CH_SENSOR_API void ChFilterSave::Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    std::shared_ptr<SensorOptixBuffer> pOptix = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    if (!pOptix && !pR8)
        throw std::runtime_error("This buffer type cannot be saved as png");

    std::string filename = m_path + "frame_" + std::to_string(pSensor->GetNumUpdates()) + ".png";

    // openGL buffers are bottom to top...so flip when writing png.
    stbi_flip_vertically_on_write(1);

    if (pOptix) {
        optix::Buffer buffer = pOptix->Buffer;

        // Query buffer information
        RTsize buffer_width_rts, buffer_height_rts;
        buffer->getSize(buffer_width_rts, buffer_height_rts);
        uint32_t width = static_cast<int>(buffer_width_rts);
        uint32_t height = static_cast<int>(buffer_height_rts);
        RTformat buffer_format = buffer->getFormat();

        int comp = 1;
        if (buffer_format == RT_FORMAT_UNSIGNED_BYTE4) {
            comp = 4;
        }
        void* imageData = buffer->map(0, RT_BUFFER_MAP_READ);

        // int stbi_write_png(char const* filename, int w, int h, int comp, const void* data, int stride_in_bytes);
        if (!stbi_write_png(filename.c_str(), width, height, comp, imageData, comp * width)) {
            std::cout << "Filed to write image: " << filename << "\n";
        }

        buffer->unmap();
    } else if (pR8) {
        char* buf = new char[pR8->Width * pR8->Height];
        cudaMemcpy(buf, pR8->Buffer.get(), pR8->Width * pR8->Height, cudaMemcpyDeviceToHost);

        // write a grayscale png
        if (!stbi_write_png(filename.c_str(), pR8->Width, pR8->Height, 1, buf, pR8->Width)) {
            std::cout << "Filed to write image: " << filename << "\n";
        }

        delete buf;
    }
}

}  // namespace sensor
}  // namespace chrono
