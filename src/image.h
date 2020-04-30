// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once
#ifndef LIBREALSENSE_IMAGE_H
#define LIBREALSENSE_IMAGE_H

#include "types.h"
#include "ctpl_stl.h"

namespace librealsense
{
    size_t           get_image_size                 (int width, int height, rs2_format format);
    int              get_image_bpp                  (rs2_format format);

    class split_frame_threadpool
    {
        public:
            ctpl::thread_pool pool;

            split_frame_threadpool() :
                pool(1)
            {
            }

            static split_frame_threadpool * instance;

            static split_frame_threadpool *get_instance () {
                if (!instance)
                    instance = new split_frame_threadpool();

                return instance;
            }
    } ;

    template<class SOURCE, class SPLIT_A, class SPLIT_B> void split_frame_parallel_sides(byte * const dest[], int count, const SOURCE * source, SPLIT_A split_a, SPLIT_B split_b)
    {
        auto a_future = split_frame_threadpool::get_instance()->pool.push([&dest, &source, count, &split_a](int id) {
            auto *a = reinterpret_cast<decltype(split_a(SOURCE())) *>(dest[0]);
            auto *s = source;
            for (int i = 0; i < count; ++i, ++s, ++a)
                *a = split_a(*s);
		});

        // use current thread for other side
        auto *b = reinterpret_cast<decltype(split_b(SOURCE())) *>(dest[1]);
        auto *s = source;
        for (int i = 0; i < count; ++i, ++s, ++b)
            *b = split_b(*s);

        a_future.wait();
    }

    template<class SOURCE, class SPLIT_A, class SPLIT_B> void split_frame_simple(byte * const dest[], int count, const SOURCE * source, SPLIT_A split_a, SPLIT_B split_b)
    {
        auto a = reinterpret_cast<decltype(split_a(SOURCE())) *>(dest[0]);
        auto b = reinterpret_cast<decltype(split_b(SOURCE())) *>(dest[1]);
        for (int i = 0; i < count; ++i)
        {
            *a++ = split_a(*source);
            *b++ = split_b(*source++);
        }
    }

    template<class SOURCE, class SPLIT_A, class SPLIT_B> void split_frame(byte * const dest[], int count, const SOURCE * source, SPLIT_A split_a, SPLIT_B split_b)
    {
        split_frame_parallel_sides<SOURCE, SPLIT_A, SPLIT_B>(dest, count, source, split_a, split_b);
    }

    resolution rotate_resolution(resolution res);
    resolution l500_confidence_resolution(resolution res);
}

#endif
