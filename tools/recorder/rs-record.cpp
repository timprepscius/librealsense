// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <memory>
#include <functional>
#include <thread>
#include <string.h>
#include <chrono>
#include "tclap/CmdLine.h"
#include <fstream>

using namespace TCLAP;

// ----------------------------------------

#pragma pack(push)

typedef uint64_t timetype;

struct FrameInfo {
    uint64_t streamNumber;
    uint64_t frameIndex;
    timetype deviceTime;
    timetype systemTime;
    uint64_t datasize;
};


struct CameraIntrinsics
{
	float focalLengthX;
	float focalLengthY;
	float principalPointX;
	float principalPointY;
	float radialDistortionSecondOrder;
	float radialDistortionFourthOrder;
	float radialDistortionSixthOrder;
	float tangentialDistortionCoefficientOne;
	float tangentialDistortionCoefficientTwo;
} ;

struct Rotation {
    float m[9];
} ;

struct Translation {
    float v[3];
} ;

struct CameraExtrinsics
{
    Rotation rotation;
    Translation translation;
} ;

struct InfraredFileHeader {
    CameraIntrinsics cameraIntrinsics;
    CameraExtrinsics cameraExtrinsics;
} ;


#pragma pack(pop)

// ----------------------------------------

std::ofstream streamFile;
std::mutex streamFileMutex;

InfraredFileHeader infraredFileHeaders[2];

// ----------------------------------------

void initialize_sensors (rs2::pipeline_profile &pipelineProfile)
{
    auto autoExposure = 0.0;
    auto exposureTime = 1024.0;
    auto exposureGain = 16.0;

    rs2::device selected_device = pipelineProfile.get_device();
    
    for (auto &sensor : selected_device.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_SHARPNESS))
        {
            sensor.set_option(RS2_OPTION_SHARPNESS, 1.0);
        }
        
        if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
        {
            sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0);
        }

        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        }

        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, autoExposure);
        }
        
        if (sensor.supports(RS2_OPTION_EXPOSURE))
        {
            sensor.set_option(RS2_OPTION_EXPOSURE, exposureTime);
        }

        if (sensor.supports(RS2_OPTION_GAIN))
        {
            sensor.set_option(RS2_OPTION_GAIN, exposureGain);
        }
    }
}

void write_headers (rs2::pipeline_profile &pipelineProfile)
{
    std::vector<rs2::video_stream_profile> infrareds;

    auto streams = pipelineProfile.get_streams();
    for (auto &s : streams)
    {
        if (s.stream_type() == RS2_STREAM_INFRARED)
        {
            infrareds.push_back(s.as<rs2::video_stream_profile>());
        }
    }

    for (int i=0; i<infrareds.size(); ++i)
    {
        std::cout << "infrared " << i << " FPS " << infrareds[0].fps() << std::endl;

        if (infrareds.size() > 1)
        {
            auto to = i == 0 ? 1 : 0;
            auto cie = infrareds[i].get_extrinsics_to(infrareds[to]);
            
            auto &t = cie.translation;
            auto &r = cie.rotation;
            
            auto &fce = infraredFileHeaders[i].cameraExtrinsics;
            fce.translation = { t[0], t[1], t[2] };
            fce.rotation = { r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8] };

            for (auto fi=0; fi<3; ++fi)
            {
                std::cout << "fce.translate[" << fi << "]" << fce.translation.v[fi] << std::endl;
            }

            for (auto fi=0; fi<9; ++fi)
            {
                std::cout << "fce.rotation[" << fi << "]" << fce.rotation.m[fi] << std::endl;
            }
        }

        auto ci = infrareds[i].get_intrinsics();
        auto &fci = infraredFileHeaders[i].cameraIntrinsics;
        fci.focalLengthX = ci.fx;
        fci.focalLengthY = ci.fy;
        fci.principalPointX = ci.ppx;
        fci.principalPointY = ci.ppy;
        
        enum COEFF { k1, k2, p1, p2, k3 };
        fci.radialDistortionSecondOrder = ci.coeffs[COEFF::k1];
        fci.radialDistortionFourthOrder = ci.coeffs[COEFF::k2];
        fci.radialDistortionSixthOrder = ci.coeffs[COEFF::k3];
        fci.tangentialDistortionCoefficientOne = ci.coeffs[COEFF::p1];
        fci.tangentialDistortionCoefficientTwo = ci.coeffs[COEFF::p2];
    }    

    for (auto i=0; i<2; ++i)
    {
        streamFile.write((char *)&infraredFileHeaders[i], sizeof(InfraredFileHeader));
    }
}

void on_frame (const rs2::frameset& frameset)
{
    std::lock_guard<std::mutex> lock(streamFileMutex);

    auto expectedFrameCount = 2;
    if (frameset.size() != expectedFrameCount)
        return;

    for (auto i =0; i<2; ++i)
    {
        auto infraredFrame = frameset.get_infrared_frame(i+1);
        uint64_t datasize = infraredFrame.get_data_size();
        auto *data = infraredFrame.get_data();

        auto multiplier = 1000000;

        auto frameNumber = infraredFrame.get_frame_number();
        timetype timestamp = timetype(infraredFrame.get_timestamp() * 1000000.0);
        timetype system_time = timetype(infraredFrame.get_system_time() * 1000000.0);

        std::cout 
            << " frameNumber " << frameNumber
            << " datasize " << datasize 
            << " timestamp "
            << infraredFrame.get_timestamp() 
            << " -> " << timestamp
            << " system-time "
            << infraredFrame.get_system_time() 
            << ", " << system_time 
            << std::endl;

        FrameInfo header {
            i,
            frameNumber,
            timestamp,
            system_time,
            datasize
        } ;

        streamFile.write((char *)&header, sizeof(header));
        streamFile.write((char *)data, datasize);
    }

    std::cout << ".";
};

int main(int argc, char * argv[]) try
{
    // Parse command line arguments
    CmdLine cmd("librealsense rs-record example tool", ' ');
    ValueArg<int>    time("t", "Time", "Amount of time to record (in seconds)", false, 10, "");
    ValueArg<std::string> out_file("f", "FullFilePath", "the file where the data will be saved to", false, "test.bag", "");

    cmd.add(time);
    cmd.add(out_file);
    cmd.parse(argc, argv);

    std::cout << "starting" << std::endl;

    streamFile.open(out_file.getValue(), std::ios::binary);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 90);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 90);
    auto expectedFrameCount = 2;

    std::cout << "sizeof FrameInfo " << sizeof(FrameInfo) << std::endl;

    rs2::pipeline_profile profiles = pipe.start(cfg);
    initialize_sensors(profiles);
    write_headers(profiles);

    auto t = std::chrono::system_clock::now();
    auto t0 = t;

    while (t - t0 <= std::chrono::seconds(time.getValue()))
    {
        auto frameset = pipe.wait_for_frames(250);
        on_frame(frameset);

        t = std::chrono::system_clock::now();
    }

    std::cout << "\nFinished" << std::endl;

    streamFile.close();    

    pipe.stop();

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
