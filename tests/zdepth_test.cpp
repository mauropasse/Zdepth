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

#include <iostream>
using namespace std;

static zdepth::DepthCompressor compressor, decompressor;

bool check_block_size()
{
    if(Width % kBlockSize != 0) {
        cout << "Failed: Width, " << Width <<  ", is not a multiple of Block Size, " << kBlockSize << endl;
        return false;
    }

    if(Height % kBlockSize != 0) {
        cout << "Failed: Height, " << Height <<  ", is not a multiple of Block Size, " << kBlockSize << endl;
        return false;
    }

    return true;
}

bool Zdepth_compress(
    const uint16_t* frame,
    bool keyframe)
{
    cout << endl;
    cout << "===================================================================" << endl;
    cout << "+ Zdepth Test: Keyframe=" << (keyframe?"Yes":"No") << endl;
    cout << "===================================================================" << endl;

    std::vector<uint8_t> compressed;
    std::vector<uint16_t> decompressed;

    const uint64_t t0 = GetTimeUsec();
    compressor.Compress(Width, Height, frame, compressed, keyframe);

    const uint64_t t1 = GetTimeUsec();

    int width, height;
    zdepth::DepthResult result = decompressor.Decompress(compressed, width, height, decompressed);

    const uint64_t t2 = GetTimeUsec();

    if (result != zdepth::DepthResult::Success) {
        cout << "Failed: decompressor.Decompress returned " << zdepth::DepthResultString(result) << endl;
        return false;
    }

    if (width != Width || height != Height)  {
        cout << "Decompression failed: Resolution" << endl;
        return false;
    }

    for (size_t i = 0; i < decompressed.size(); ++i) {
        if (AzureKinectQuantizeDepth(decompressed[i]) != AzureKinectQuantizeDepth(frame[i])) {
            cout << "Decompression failed: Contents did not match at offset = " << i << endl;
            return false;
        }
    }

    const unsigned original_bytes = Width * Height * 2;

    cout << endl;
    cout << "Zdepth Compression: " << original_bytes << " bytes -> " << compressed.size() <<
        " bytes (ratio = " << original_bytes / (float)compressed.size() << ":1) ("
        << (compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
    cout << "Zdepth Speed: Compressed in " << (t1 - t0) / 1000.f << " msec. Decompressed in " << (t2 - t1) / 1000.f << " msec" << endl;
    cout << "Compress + Decompress: " << (t2 - t0) / 1000.f << " msec." << std::endl;

    return true;
}

bool RVL_compress(
    const uint16_t* frame,
    bool do_quantize,
    bool do_zstd_compress)
{
    cout << endl;
    cout << "===================================================================" << endl;
    cout << "+ RVL Test: Cuantize= " << (do_quantize?"Yes":"No")
         << " / Zstd=" << (do_zstd_compress?"Yes":"No") << endl;
    cout << "===================================================================" << endl;

    // Vectors
    std::vector<uint8_t> compressed;
    std::vector<uint8_t> zstd_compressed;

    const int total_pixels = Width * Height;
    const unsigned original_bytes = total_pixels * 2;
    std::vector<uint16_t> quantized(total_pixels);
    std::vector<uint16_t> decompressed(total_pixels);

    // Timestamps

    uint64_t quantize_time = 0;
    uint64_t dequantize_time = 0;
    uint64_t rvl_compress_time = 0;
    uint64_t rvl_decompress_time = 0;
    uint64_t zstd_compress_time = 0;
    uint64_t zstd_decompress_time = 0;

    int compressed_bytes = 0;
    uint64_t t0 = GetTimeUsec();

    if (do_quantize) {
      QuantizeDepthImage(Width, Height, frame, quantized);
      uint64_t t1 = GetTimeUsec();
      quantize_time = t1 - t0;

      compressed.resize(total_pixels * 3); // Times three because ..?
      compressed_bytes = CompressRVL((short*)quantized.data(), (char*)compressed.data(), total_pixels);
      uint64_t t2 = GetTimeUsec();
      rvl_compress_time = t2 - t1;
    } else {
      compressed.resize(total_pixels * 2); // Times two because pixels are 16bits and compressed 8bits
      compressed_bytes = CompressRVL((short*)frame, (char*)compressed.data(), total_pixels);
      rvl_compress_time = GetTimeUsec() - t0;
    }
    compressed.resize(compressed_bytes);

    uint64_t t1 = GetTimeUsec();

    // Extra Zstd compression/decompression
    if (do_zstd_compress) {
        // Compress
        ZstdCompress(compressed, zstd_compressed);
        uint64_t t2 = GetTimeUsec();
        zstd_compress_time = t2 - t1;

        // Decompress
        ZstdDecompress(zstd_compressed.data(), zstd_compressed.size(), compressed.size(), compressed);
        zstd_decompress_time = GetTimeUsec() - t2;
    }

    uint64_t t2 = GetTimeUsec();

    if (do_quantize) {
        DecompressRVL((char*)compressed.data(), (short*)decompressed.data(), total_pixels);
        uint64_t t3 = GetTimeUsec();
        rvl_decompress_time = t3 - t2;

        quantized.resize(total_pixels);
        DequantizeDepthImage(Width, Height, quantized.data(), decompressed);
        dequantize_time = GetTimeUsec() - t3;
    } else {
        DecompressRVL((char*)compressed.data(), (short*)decompressed.data(), total_pixels);
        rvl_decompress_time = GetTimeUsec() - t2;
    }

    if (do_quantize) {
        for (size_t i = 0; i < decompressed.size(); ++i) {
            if (AzureKinectQuantizeDepth(decompressed[i]) != AzureKinectQuantizeDepth(frame[i])) {
                cout << "Decompression failed: Contents did not match at offset = " << i << endl;
                return false;
            }
        }
    } else {
        for (size_t i = 0; i < decompressed.size(); ++i) {
            if (decompressed[i] != frame[i]) {
                cout << "Decompression failed: Contents did not match at offset = " << i << endl;
                return false;
            }
        }
    }


    cout << endl;

    if (do_zstd_compress) {
        if (do_quantize) {
            cout << "Quantization+RVL+Zstd Compression: " << original_bytes << " bytes -> " << zstd_compressed.size()
                 << " bytes (ratio = " << original_bytes / (float)zstd_compressed.size() << ":1) ("
                 << (zstd_compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
            cout << "Quantization time: " << quantize_time / 1000.f << " msec. Dequantized in " << dequantize_time / 1000.f << " msec" << std::endl;
        } else {
            cout << "RVL+Zstd Compression: " << original_bytes << " bytes -> " << zstd_compressed.size()
                 << " bytes (ratio = " << original_bytes / (float)zstd_compressed.size() << ":1) ("
                 << (zstd_compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
        }
        cout << "Zstd compress time: " << zstd_compress_time / 1000.f << " msec. Zstd decompress in " << zstd_decompress_time / 1000.f << " msec" << std::endl;
    } else {
        if (do_quantize) {
            cout << "Quantization+RVL Compression: " << original_bytes << " bytes -> " << compressed.size()
                 << " bytes (ratio = " << original_bytes / (float)compressed.size() << ":1) ("
                 << (compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
            cout << "Quantization time: " << quantize_time / 1000.f << " msec. Dequantized in " << dequantize_time / 1000.f << " msec" << std::endl;
        } else {
            cout << "RVL Compression: " << original_bytes << " bytes -> " << compressed.size()
                 << " bytes (ratio = " << original_bytes / (float)compressed.size() << ":1)"
                 << " (" << (compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
        }
    }

    cout << "RVL compress time: " <<  rvl_compress_time / 1000.f << " msec. RVL decompress in " << rvl_decompress_time / 1000.f << " msec" << std::endl;

    uint64_t total_compress = quantize_time + zstd_compress_time + rvl_compress_time;
    uint64_t total_decompress = dequantize_time + zstd_decompress_time + rvl_decompress_time;
    cout << endl;
    cout << "Total compress: " << total_compress / 1000.f << " msec. Total decompress: " << total_decompress / 1000.f << " msec" << std::endl;
    cout << "Compress + Decompress: " << (total_compress + total_decompress) / 1000.f << " msec." << std::endl;

    cout << endl;
    return true;
}

void print_test_header(bool keyframe, bool do_quantize, bool do_zstd_compress)
{
}

bool TestPattern(const uint16_t* frame0)
{
    bool keyframe = true;
    if (!Zdepth_compress(frame0, keyframe)) {
        cout << "Failure: frame0 failed";
        return false;
    }

    keyframe = false;
    if (!Zdepth_compress(frame0, keyframe)) {
        cout << "Failure: frame0 failed";
        return false;
    }

    bool do_quantize = false;
    bool do_zstd_compress = false;

    for (int i=0; i < 10; i++) {
        if (!RVL_compress(frame0, do_quantize, do_zstd_compress)) {
            cout << "Failure: frame0 failed";
            return false;
        }
    }

    do_quantize = false;
    do_zstd_compress = true;
    if (!RVL_compress(frame0, do_quantize, do_zstd_compress)) {
        cout << "Failure: frame0 failed";
        return false;
    }

    do_quantize = true;
    do_zstd_compress = false;
    if (!RVL_compress(frame0, do_quantize, do_zstd_compress)) {
        cout << "Failure: frame0 failed";
        return false;
    }

    do_quantize = true;
    do_zstd_compress = true;
    if (!RVL_compress(frame0, do_quantize, do_zstd_compress)) {
        cout << "Failure: frame0 failed";
        return false;
    }

    return true;
}

int main(int argc, char* argv[])
{
    (void)(argc);
    (void)(argv);

    cout << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << "Test vector: Room 0" << endl;
    cout << "-------------------------------------------------------------------" << endl;

    if (!TestPattern(TestVector0_Room0)) {
        cout << "Test failure: Room test vector" << endl;
        return -1;
    }
    cout << "Test success" << endl;
    return 0;
}
