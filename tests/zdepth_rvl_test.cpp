// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"
using namespace zdepth;


//------------------------------------------------------------------------------
// Timing

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#elif __MACH__
    #include <sys/file.h>
    #include <mach/mach_time.h>
    #include <mach/mach.h>
    #include <mach/clock.h>

    extern mach_port_t clock_port;
#else
    #include <time.h>
    #include <sys/time.h>
    #include <sys/file.h> // flock
#endif

#ifdef _WIN32
// Precomputed frequency inverse
static double PerfFrequencyInverseUsec = 0.;
static double PerfFrequencyInverseMsec = 0.;

static void InitPerfFrequencyInverse()
{
    LARGE_INTEGER freq = {};
    if (!::QueryPerformanceFrequency(&freq) || freq.QuadPart == 0) {
        return;
    }
    const double invFreq = 1. / (double)freq.QuadPart;
    PerfFrequencyInverseUsec = 1000000. * invFreq;
    PerfFrequencyInverseMsec = 1000. * invFreq;
}
#elif __MACH__
static bool m_clock_serv_init = false;
static clock_serv_t m_clock_serv = 0;

static void InitClockServ()
{
    m_clock_serv_init = true;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &m_clock_serv);
}
#endif // _WIN32

uint64_t GetTimeUsec()
{
#ifdef _WIN32
    LARGE_INTEGER timeStamp = {};
    if (!::QueryPerformanceCounter(&timeStamp)) {
        return 0;
    }
    if (PerfFrequencyInverseUsec == 0.) {
        InitPerfFrequencyInverse();
    }
    return (uint64_t)(PerfFrequencyInverseUsec * timeStamp.QuadPart);
#elif __MACH__
    if (!m_clock_serv_init) {
        InitClockServ();
    }

    mach_timespec_t tv;
    clock_get_time(m_clock_serv, &tv);

    return 1000000 * tv.tv_sec + tv.tv_nsec / 1000;
#else
    // This seems to be the best clock to used based on:
    // http://btorpey.github.io/blog/2014/02/18/clock-sources-in-linux/
    // The CLOCK_MONOTONIC_RAW seems to take a long time to query,
    // and so it only seems useful for timing code that runs a small number of times.
    // The CLOCK_MONOTONIC is affected by NTP at 500ppm but doesn't make sudden jumps.
    // Applications should already be robust to clock skew so this is acceptable.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_nsec / 1000) + static_cast<uint64_t>(ts.tv_sec) * 1000000;
#endif
}


//------------------------------------------------------------------------------
// RVL

// RVL library for performance baseline

// Paper: https://www.microsoft.com/en-us/research/publication/fast-lossless-depth-image-compression/
// Video presentation: https://www.youtube.com/watch?v=WYU2upBs2hA

// RVL author suggests that H.264 is a bad idea to use.
// But it seems like some masking can be used to avoid messing up the edges...

// Effective Compression of Range Data Streams for Remote Robot Operations using H.264
// http://www2.informatik.uni-freiburg.de/~stachnis/pdf/nenci14iros.pdf

// Adapting Standard Video Codecs for Depth Streaming
// http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf

inline void EncodeVLE(int* &pBuffer, int& word, int& nibblesWritten, int value)
{
    do
    {
        int nibble = value & 0x7; // lower 3 bits
        if (value >>= 3) {
            nibble |= 0x8; // more to come
        }
        word <<= 4;
        word |= nibble;
        if (++nibblesWritten == 8) // output word
        {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

inline int DecodeVLE(int* &pBuffer, int& word, int& nibblesWritten)
{
    unsigned int nibble;
    int value = 0, bits = 29;
    do
    {
        if (!nibblesWritten)
        {
            word = *pBuffer++; // load word
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

int CompressRVL(short* input, char* output, int numPixels)
{
    int word = 0;
    int nibblesWritten = 0;
    int *pBuffer;
    int *buffer = pBuffer = (int*)output;
    short *end = input + numPixels;
    short previous = 0;
    while (input != end)
    {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++);
        EncodeVLE(pBuffer, word, nibblesWritten, zeros); // number of zeros
        for (short* p = input; (p != end) && *p++; nonzeros++);
        EncodeVLE(pBuffer, word, nibblesWritten, nonzeros); // number of nonzeros
        for (int i = 0; i < nonzeros; i++)
        {
            short current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(pBuffer, word, nibblesWritten, positive); // nonzero value
            previous = current;
        }
    }
    if (nibblesWritten) // last few values
    {
        *pBuffer++ = word << 4 * (8 - nibblesWritten);
    }
    return int((char*)pBuffer - (char*)buffer); // num bytes
}

void DecompressRVL(char* input, short* output, int numPixels)
{
    int word, nibblesWritten;
    int * pBuffer = (int*)input;
    nibblesWritten = 0;
    short current, previous = 0;
    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode)
    {
        int zeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--) {
            *output++ = 0;
        }
        int nonzeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--)
        {
            int positive = DecodeVLE(pBuffer, word, nibblesWritten); // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}


//------------------------------------------------------------------------------
// Test Vectors

#include "test_vectors.inl"


//------------------------------------------------------------------------------
// Test Application

#include <algorithm>
#include <cassert>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <pthread.h>
#include <set>
#include <vector>

using namespace std;
#define DEBUG_MODE

static zdepth::DepthCompressor compressor, decompressor;

struct TimeStats
{
    std::string frame_name;
    size_t original_bytes = 0;
    size_t compressed_size = 0;
    float compression_ratio = 0;
    uint64_t quantize_time = 0;
    uint64_t dequantize_time = 0;
    uint64_t rvl_compress_time = 0;
    uint64_t rvl_decompress_time = 0;
    uint64_t zstd_compress_time = 0;
    uint64_t zstd_decompress_time = 0;
    uint64_t zdepth_compress_time = 0;
    uint64_t zdepth_decompress_time = 0;
    uint64_t total_compress = 0;
    uint64_t total_decompress = 0;
    uint64_t total = 0;
};

void print_stats(TimeStats * stats, size_t n_stats)
{
    for(size_t i = 0; i < n_stats; i++) {
        cout << std::left << std::setw(20) << std::setfill(' ') << stats[i].frame_name;
        cout << std::left << std::setw(20) << std::setfill(' ') << stats[i].original_bytes;
        cout << std::left << std::setw(20) << std::setfill(' ') << stats[i].compressed_size;
        cout << std::left << std::setw(20) << std::setfill(' ') << stats[i].compression_ratio;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].zdepth_compress_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].zdepth_decompress_time / 1000.f;;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].quantize_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].dequantize_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].rvl_compress_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].rvl_decompress_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].zstd_compress_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].zstd_decompress_time / 1000.f;
        cout << std::left << std::setw(15) << std::setfill(' ') << stats[i].total / 1000.f << endl;
    }
}

TimeStats Zdepth_compress(
    const uint16_t * frame,
    bool keyframe,
    int width,
    int height)
{
    size_t total_pixels = width * height;
    std::vector<uint8_t> compressed;
    std::vector<uint16_t> decompressed(total_pixels);

    const uint64_t t0 = GetTimeUsec();
    compressor.Compress(width, height, frame, compressed, keyframe);
    const uint64_t t1 = GetTimeUsec();

    int other_width, other_height;
    zdepth::DepthResult result = decompressor.Decompress(compressed, other_width, other_height, decompressed);

    const uint64_t t2 = GetTimeUsec();

    if (result != zdepth::DepthResult::Success) {
        cout << "Failed: decompressor.Decompress returned " << zdepth::DepthResultString(result) << endl;
        assert(0);
    }

    // if (width != other_width || height != other_height)  {
    //     cout << "Decompression failed: Resolution mismatch" << endl;
    //     return false;
    // }

    // for (size_t i = 0; i < decompressed.size(); ++i) {
    //     if (AzureKinectQuantizeDepth(decompressed[i]) != AzureKinectQuantizeDepth(frame[i])) {
    //         cout << "Decompression failed: Contents did not match at offset = " << i << endl;
    //         return false;
    //     }
    // }

    TimeStats stats;
    stats.original_bytes = total_pixels * 2;
    stats.compressed_size = compressed.size();
    stats.compression_ratio = stats.original_bytes / (float)compressed.size();
    stats.zdepth_compress_time = t1 - t0;
    stats.zdepth_decompress_time = t2 - t1;
    stats.total = t2 - t0;

    return stats;
}

TimeStats RVL_compress(
    const uint16_t* frame,
    bool do_quantize,
    bool do_zstd_compress,
    int width,
    int height)
{
    size_t total_pixels = width * height;
    std::vector<uint16_t> quantized(total_pixels);
    std::vector<uint16_t> decompressed(total_pixels);
    std::vector<uint8_t> zstd_compressed;
    std::vector<uint8_t> compressed;

    TimeStats stats;

    int compressed_bytes = 0;
    uint64_t t0 = GetTimeUsec();

    if (do_quantize) {
      QuantizeDepthImage(width, height, frame, quantized);
      uint64_t t1 = GetTimeUsec();
      stats.quantize_time = t1 - t0;

      compressed.resize(total_pixels * 3); // Times three because ..?
      compressed_bytes = CompressRVL((short*)quantized.data(), (char*)compressed.data(), total_pixels);
      uint64_t t2 = GetTimeUsec();
      stats.rvl_compress_time = t2 - t1;
    } else {
      compressed.resize(total_pixels * 2); // Times two because pixels are 16bits and compressed 8bits
      compressed_bytes = CompressRVL((short*)frame, (char*)compressed.data(), total_pixels);
      stats.rvl_compress_time = GetTimeUsec() - t0;
    }
    compressed.resize(compressed_bytes);

    uint64_t t1 = GetTimeUsec();

    // Extra Zstd compression/decompression
    if (do_zstd_compress) {
        // Compress
        ZstdCompress(compressed, zstd_compressed);
        uint64_t t2 = GetTimeUsec();
        stats.zstd_compress_time = t2 - t1;

        // Decompress
        ZstdDecompress(zstd_compressed.data(), zstd_compressed.size(), compressed.size(), compressed);
        stats.zstd_decompress_time = GetTimeUsec() - t2;
    }

    uint64_t t2 = GetTimeUsec();

    if (do_quantize) {
        DecompressRVL((char*)compressed.data(), (short*)decompressed.data(), total_pixels);
        uint64_t t3 = GetTimeUsec();
        stats.rvl_decompress_time = t3 - t2;

        quantized.resize(total_pixels);
        DequantizeDepthImage(Width, Height, quantized.data(), decompressed);
        stats.dequantize_time = GetTimeUsec() - t3;
    } else {
        DecompressRVL((char*)compressed.data(), (short*)decompressed.data(), total_pixels);
        stats.rvl_decompress_time = GetTimeUsec() - t2;
    }

    // This check has overhead. Remove in testing
    // if (do_quantize) {
    //     for (size_t i = 0; i < decompressed.size(); ++i) {
    //         if (AzureKinectQuantizeDepth(decompressed[i]) != AzureKinectQuantizeDepth(frame[i])) {
    //             cout << "Decompression failed: Contents did not match at offset = " << i << endl;
    //         }
    //     }
    // } else {
    //     for (size_t i = 0; i < decompressed.size(); ++i) {
    //         if (decompressed[i] != frame[i]) {
    //             cout << "Decompression failed: Contents did not match at offset = " << i << endl;
    //         }
    //     }
    // }

    size_t compressed_size;
    float compression_ratio;
    size_t original_bytes = total_pixels * 2;
    if (do_zstd_compress) {
        if (do_quantize) {
            compressed_size = zstd_compressed.size();
            compression_ratio = original_bytes / (float)compressed_size;
        } else {
            compressed_size = zstd_compressed.size();
            compression_ratio = original_bytes / (float)compressed_size;
        }
    } else {
        if (do_quantize) {
            compressed_size = compressed.size();
            compression_ratio = original_bytes / (float)compressed_size;
        } else {
            compressed_size = compressed.size();
            compression_ratio = original_bytes / (float)compressed_size;
        }
    }

    stats.total_compress = stats.quantize_time + stats.zstd_compress_time + stats.rvl_compress_time;
    stats.total_decompress = stats.dequantize_time + stats.zstd_decompress_time + stats.rvl_decompress_time;
    stats.total = stats.total_compress + stats.total_compress;
    stats.original_bytes = original_bytes;
    stats.compressed_size = compressed_size;
    stats.compression_ratio = compression_ratio;

    return stats;
}

enum class Algorithm
{
    RVL,
    RVL_QUANTIZE,
    RVL_ZSTD,
    RVL_QUANTIZE_ZSTD,
    ZDEPTH_KEY_FRAME,
    ZDEPTH_NO_KEY_FRAME
};

bool CompressFrame(
    const std::string & frame_name,
    const uint16_t * frame,
    int width,
    int height,
    Algorithm compression_algorithm,
    TimeStats & average_stats)
{
    int iterations_per_image = 1;

    TimeStats total_stats;

    for (int i=0; i < iterations_per_image; i++) {
        TimeStats stats;

        switch (compression_algorithm)
        {
            case Algorithm::RVL:
                stats = RVL_compress(frame, false, false, width, height);
                break;

            case Algorithm::RVL_QUANTIZE:
                stats = RVL_compress(frame, true, false, width, height);
                break;

            case Algorithm::RVL_ZSTD:
                stats = RVL_compress(frame, false, true, width, height);
                break;

            case Algorithm::RVL_QUANTIZE_ZSTD:
                stats = RVL_compress(frame, true, true, width, height);
                break;

            case Algorithm::ZDEPTH_KEY_FRAME:
                stats = Zdepth_compress(frame, true, width, height);
                break;

            case Algorithm::ZDEPTH_NO_KEY_FRAME:
                stats = Zdepth_compress(frame, false, width, height);
                break;

            default: break;
        }

        total_stats.compression_ratio = stats.compression_ratio;
        total_stats.compressed_size = stats.compressed_size;
        total_stats.zdepth_compress_time += stats.zdepth_compress_time;
        total_stats.zdepth_decompress_time += stats.zdepth_decompress_time;
        total_stats.quantize_time += stats.quantize_time;
        total_stats.dequantize_time += stats.dequantize_time;
        total_stats.rvl_compress_time += stats.rvl_compress_time;
        total_stats.rvl_decompress_time += stats.rvl_decompress_time;
        total_stats.zstd_compress_time += stats.zstd_compress_time;
        total_stats.zstd_decompress_time += stats.zstd_decompress_time;
        total_stats.total += stats.total;
    }

    average_stats.frame_name = frame_name;
    average_stats.compressed_size = total_stats.compressed_size;
    average_stats.compression_ratio = total_stats.compression_ratio;
    average_stats.original_bytes = width * height * 2;
    average_stats.zdepth_compress_time = total_stats.zdepth_compress_time / iterations_per_image;
    average_stats.zdepth_decompress_time = total_stats.zdepth_decompress_time / iterations_per_image;
    average_stats.quantize_time = total_stats.quantize_time / iterations_per_image;
    average_stats.dequantize_time = total_stats.dequantize_time / iterations_per_image;
    average_stats.rvl_compress_time = total_stats.rvl_compress_time / iterations_per_image;
    average_stats.rvl_decompress_time = total_stats.rvl_decompress_time / iterations_per_image;
    average_stats.zstd_compress_time = total_stats.zstd_compress_time / iterations_per_image;
    average_stats.zstd_decompress_time = total_stats.zstd_decompress_time / iterations_per_image;
    average_stats.total = total_stats.total / iterations_per_image;

    return true;
}

void print_header(Algorithm compression_algorithm)
{
    std::string algo;
    switch (compression_algorithm) {
        case Algorithm::RVL: algo = "RVL"; break;
        case Algorithm::RVL_QUANTIZE: algo = "RVL+Q"; break;
        case Algorithm::RVL_ZSTD: algo = "RVL+Zstd"; break;
        case Algorithm::RVL_QUANTIZE_ZSTD: algo = "RVL+Q+Zstd"; break;
        case Algorithm::ZDEPTH_KEY_FRAME: algo = "ZK=1"; break;
        case Algorithm::ZDEPTH_NO_KEY_FRAME: algo = "ZK=0"; break;
    }
    cout << endl;
    cout << std::left << std::setw(20) << std::setfill(' ') << "Frame_name";
    cout << std::left << std::setw(20) << std::setfill(' ') << "Raw_B_" + algo;
    cout << std::left << std::setw(20) << std::setfill(' ') << "Comp_B_" + algo;
    cout << std::left << std::setw(20) << std::setfill(' ') << "Ratio_"+ algo;
    cout << std::left << std::setw(15) << std::setfill(' ') << "Zdepth_C";
    cout << std::left << std::setw(15) << std::setfill(' ') << "Zdepth_D";
    cout << std::left << std::setw(15) << std::setfill(' ') << "Quantize_C";
    cout << std::left << std::setw(15) << std::setfill(' ') << "Quantize_D";
    cout << std::left << std::setw(15) << std::setfill(' ') << "RVL_C";
    cout << std::left << std::setw(15) << std::setfill(' ') << "RVL_D";
    cout << std::left << std::setw(15) << std::setfill(' ') << "Zstd_C";
    cout << std::left << std::setw(15) << std::setfill(' ') << "Zstd_D";
    cout << std::left << std::setw(15) << std::setfill(' ') << "Total_" + algo << endl;
}

int test_single_frames(Algorithm compression_algorithm)
{
    print_header(compression_algorithm);

    // uint16_t width = Width;
    // uint16_t height = Height;

    // if (!CompressFrame("Room0", TestVector0_Room0, width, height, compression_algorithm)) {
    //     return -1;
    // }
    // if (!CompressFrame("Room1", TestVector0_Room1, width, height, compression_algorithm)) {
    //     return -1;
    // }
    // if (!CompressFrame("Ceiling0", TestVector1_Ceiling0, width, height, compression_algorithm)) {
    //     return -1;
    // }
    // if (!CompressFrame("Ceiling1", TestVector1_Ceiling1, width, height, compression_algorithm)) {
    //     return -1;
    // }
    // if (!CompressFrame("Person0", TestVector2_Person0, width, height, compression_algorithm)) {
    //     return -1;
    // }
    // if (!CompressFrame("Person1", TestVector2_Person1, width, height, compression_algorithm)) {
    //     return -1;
    // }
    return 0;
}

int test_image_stream(Algorithm compression_algorithm)
{
    print_header(compression_algorithm);

    // Frame specs

    DIR *dir;
    struct dirent * diread;
    vector<std::string> files;

    if ((dir = opendir("/home/mauro/irobot/sidereal/compression_algorithms/Zdepth/scripts/sim_depth_frames/")) != nullptr) {
        while ((diread = readdir(dir)) != nullptr) {
            files.push_back(diread->d_name);
        }
        closedir (dir);
    } else {
        perror ("opendir");
        return EXIT_FAILURE;
    }

    // Sort frames by name
    std::sort(files.begin(), files.end());

    // Compress frames
    for (const auto & entry : files) {
        // Frame params
        // Resolutions:
        //  640x360 (eYs3D), 640x480 (sim) and 848x480 (realsense)
        constexpr uint16_t width = 640;
        constexpr uint16_t height = 480;
        constexpr size_t total_pixeles = width * height;
        uint16_t depth_image[total_pixeles];

        // Get frame full name
        std::string full_name = "/home/mauro/irobot/sidereal/compression_algorithms/Zdepth/scripts/sim_depth_frames/" + entry;

        // Copy binary frame to image array
        FILE* binary_depth_image = fopen(full_name.c_str(), "rb");
        size_t ret = fread(depth_image, sizeof(uint16_t), total_pixeles, binary_depth_image);
        fclose(binary_depth_image);

        // Skip empty files
        if (!ret) {
          continue;
        }

        constexpr uint8_t number_of_threads = 8;

        if (total_pixeles % number_of_threads) {
            cout << "Total pixeles not divisible in " << number_of_threads << " threads." << endl;
            return -1;
        }

        uint16_t * sub_frames[number_of_threads];
        TimeStats sub_frames_stats[number_of_threads];

        for (size_t i = 0; i < number_of_threads; i++) {
          size_t offset = (total_pixeles / number_of_threads) * i;
          sub_frames[i] = depth_image + offset;
        }

        constexpr uint16_t sub_frames_height = height / number_of_threads;

        std::thread t0([&](){CompressFrame(entry, sub_frames[0], width, sub_frames_height, compression_algorithm, sub_frames_stats[0]);});
        std::thread t1([&](){CompressFrame(entry, sub_frames[1], width, sub_frames_height, compression_algorithm, sub_frames_stats[1]);});
        std::thread t2([&](){CompressFrame(entry, sub_frames[2], width, sub_frames_height, compression_algorithm, sub_frames_stats[2]);});
        std::thread t3([&](){CompressFrame(entry, sub_frames[3], width, sub_frames_height, compression_algorithm, sub_frames_stats[3]);});
        std::thread t4([&](){CompressFrame(entry, sub_frames[4], width, sub_frames_height, compression_algorithm, sub_frames_stats[4]);});
        std::thread t5([&](){CompressFrame(entry, sub_frames[5], width, sub_frames_height, compression_algorithm, sub_frames_stats[5]);});
        std::thread t6([&](){CompressFrame(entry, sub_frames[6], width, sub_frames_height, compression_algorithm, sub_frames_stats[6]);});
        std::thread t7([&](){CompressFrame(entry, sub_frames[7], width, sub_frames_height, compression_algorithm, sub_frames_stats[7]);});

        t0.join();
        t1.join();
        t2.join();
        t3.join();
        t4.join();
        t5.join();
        t6.join();
        t7.join();

        print_stats(sub_frames_stats, number_of_threads);

        // Compress frame
        // if (!CompressFrame(entry, depth_image, width, height, compression_algorithm)) {
        //  return -1;
        // }
    }
    return 0;
}



int main(int argc, char* argv[])
{
    (void)(argc);
    (void)(argv);

    // test_single_frames(Algorithm::RVL);
    test_image_stream(Algorithm::RVL);
    test_image_stream(Algorithm::RVL_QUANTIZE);
    test_image_stream(Algorithm::RVL_ZSTD);
    test_image_stream(Algorithm::RVL_QUANTIZE_ZSTD);
    test_image_stream(Algorithm::ZDEPTH_KEY_FRAME);
    test_image_stream(Algorithm::ZDEPTH_NO_KEY_FRAME);

    return 0;
}
