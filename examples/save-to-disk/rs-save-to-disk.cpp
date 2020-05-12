// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    std::cout << "starting" << std::endl;

    auto resolution = 848;
    int requestedWidth, requestedHeight, requestedFPS;
    if (resolution==848)
    {
        requestedWidth = 848;
        requestedHeight = 480;
        requestedFPS = 90;
    }
    else
    if (resolution==1280)
    {
        requestedWidth = 1280;
        requestedHeight = 720;
        requestedFPS = 30;
    }
    else
    {
        std::cout << "resolution not found" << std::endl;
        return 1;
    }

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, requestedWidth, requestedHeight, RS2_FORMAT_Y8, requestedFPS);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, requestedWidth, requestedHeight, RS2_FORMAT_Y8, requestedFPS);
    auto expectedFrameCount = 2;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    // rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    auto pipelineProfile = pipe.start(cfg);

    rs2::device selected_device = pipelineProfile.get_device();

    for (auto &sensor : selected_device.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        }        
    }

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
    // has settled, we will write these to disk
    for (auto&& frame : pipe.wait_for_frames())
    {
        // We can only save video frames as pngs, so we skip the rest
        if (auto vf = frame.as<rs2::video_frame>())
        {
            auto stream = frame.get_profile().stream_type();
            // Use the colorizer to get an rgb image for the depth stream
            if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

            // Write images to disk
            std::stringstream png_file;
            png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                           vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout << "Saved " << png_file.str() << std::endl;

            // Record per-frame metadata for UVC streams
            std::stringstream csv_file;
            csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
                     << "-metadata.csv";
            metadata_to_csv(vf, csv_file.str());
        }
    }

    return EXIT_SUCCESS;
}
catch(const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
