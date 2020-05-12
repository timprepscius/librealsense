// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default

/*
    int requestedWidth = 848;
    int requestedHeight = 480;
    int requestedFPS = 90;
*/
    int requestedWidth = 1280;
    int requestedHeight = 720;
    int requestedFPS = 30;


    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, requestedWidth, requestedHeight, RS2_FORMAT_Y8, requestedFPS);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, requestedWidth, requestedHeight, RS2_FORMAT_Y8, requestedFPS);

    auto pipelineProfile = pipe.start(cfg);
    rs2::device selected_device = pipelineProfile.get_device();

    for (auto &sensor : selected_device.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        }        
    }


    while (app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                             apply_filter(printer).     // Print each enabled stream frame rate
                             apply_filter(color_map);   // Find and colorize the depth data

        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        app.show(data);

        for (auto frame : data)
        {
            if (auto vf = frame.as<rs2::video_frame>())
            {
                auto frameIndex = frame.get_frame_number();
                std::stringstream png_file;
                png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << "_" << frameIndex << ".png";
                stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                                vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            }
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}