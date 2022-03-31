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
