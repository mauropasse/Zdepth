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

#include "test_vectors.inl"
#include "zdepth_rvl_test.hpp"
#include "depth_frame.hpp"

void
Zdepth_compress(
    DepthFrame & frame,
    bool keyframe,
    zdepth::DepthCompressor & compressor,
    zdepth::DepthCompressor & decompressor)
{
    FrameStats & stats = frame.stats_;
    std::vector<uint8_t> compressed;
    std::vector<uint16_t> decompressed(frame.get_frame_pixels());

    // const uint64_t t0 = GetTimeUsec();
    auto res = compressor.Compress(frame.width_, frame.height_, frame.data_, compressed, keyframe);

    if (res != zdepth::DepthResult::Success) {
        cout << "Failed: decompressor.Decompress returned " << zdepth::DepthResultString(res) << endl;
        assert(0);
    }

    // const uint64_t t1 = GetTimeUsec();

    int out_width, out_height;
    res = decompressor.Decompress(compressed, out_width, out_height, decompressed);

    // const uint64_t t2 = GetTimeUsec();

    if (res != zdepth::DepthResult::Success) {
        cout << "Failed: decompressor.Decompress returned " << zdepth::DepthResultString(res) << endl;
        assert(0);
    }

    // if (width != out_width || height != out_height)  {
    //     cout << "Decompression failed: Resolution mismatch" << endl;
    //     return false;
    // }

    // for (size_t i = 0; i < decompressed.size(); ++i) {
    //     if (AzureKinectQuantizeDepth(decompressed[i]) != AzureKinectQuantizeDepth(frame.data_[i])) {
    //         cout << "Decompression failed: Contents did not match at offset = " << i << endl;
    //         return false;
    //     }
    // }

    stats.uncompressed_size = frame.get_frame_bytes();
    stats.compressed_size = compressed.size();
    stats.compression_ratio = stats.uncompressed_size / (float)compressed.size();
    // stats.zdepth_compress_time = t1 - t0;
    // stats.zdepth_decompress_time = t2 - t1;
    // stats.total = t2 - t0;
}

void
RVL_compress(
    DepthFrame & frame,
    bool do_quantize,
    bool do_zstd_compress)
{
    int total_pixels = frame.get_frame_pixels();
    std::vector<uint16_t> quantized(total_pixels);
    std::vector<uint16_t> decompressed(total_pixels);
    std::vector<uint8_t> zstd_compressed;
    std::vector<uint8_t> compressed;

    int compressed_bytes = 0;
    int width = frame.width_;
    int height = frame.height_;

    FrameStats & stats = frame.stats_;

    // uint64_t t0 = GetTimeUsec();

    if (do_quantize) {
      QuantizeDepthImage(width, height, frame.data_, quantized);
      // uint64_t t1 = GetTimeUsec();
      // stats.quantize_time = t1 - t0;

      compressed.resize(total_pixels * 3); // Times three because ..?
      compressed_bytes = CompressRVL((short*)quantized.data(), (char*)compressed.data(), total_pixels);
      // uint64_t t2 = GetTimeUsec();
      // stats.rvl_compress_time = t2 - t1;
    } else {
      compressed.resize(total_pixels * 2); // Times two because pixels are 16bits and compressed 8bits
      compressed_bytes = CompressRVL((short*)frame.data_, (char*)compressed.data(), total_pixels);
      // stats.rvl_compress_time = GetTimeUsec() - t0;
    }
    compressed.resize(compressed_bytes);

    // uint64_t t1 = GetTimeUsec();

    // Extra Zstd compression/decompression
    if (do_zstd_compress) {
        // Compress
        ZstdCompress(compressed, zstd_compressed);
        // uint64_t t2 = GetTimeUsec();
        // stats.zstd_compress_time = t2 - t1;

        // Decompress
        ZstdDecompress(zstd_compressed.data(), zstd_compressed.size(), compressed.size(), compressed);
        // stats.zstd_decompress_time = GetTimeUsec() - t2;
    }

    // uint64_t t2 = GetTimeUsec();

    if (do_quantize) {
        DecompressRVL((char*)compressed.data(), (short*)decompressed.data(), total_pixels);
        // uint64_t t3 = GetTimeUsec();
        // stats.rvl_decompress_time = t3 - t2;

        quantized.resize(total_pixels);
        DequantizeDepthImage(width, height, quantized.data(), decompressed);
        // stats.dequantize_time = GetTimeUsec() - t3;
    } else {
        DecompressRVL((char*)compressed.data(), (short*)decompressed.data(), total_pixels);
        // stats.rvl_decompress_time = GetTimeUsec() - t2;
    }

    // This check has overhead. Remove in testing
    // if (do_quantize) {
    //     for (size_t i = 0; i < decompressed.size(); ++i) {
    //         if (AzureKinectQuantizeDepth(decompressed[i]) != AzureKinectQuantizeDepth(frame.data_[i])) {
    //             cout << "Decompression failed: Contents did not match at offset = " << i << endl;
    //         }
    //     }
    // } else {
    //     for (size_t i = 0; i < decompressed.size(); ++i) {
    //         if (decompressed[i] != frame.data_[i]) {
    //             cout << "Decompression failed: Contents did not match at offset = " << i << endl;
    //         }
    //     }
    // }

    size_t compressed_size;
    float compression_ratio;
    size_t uncompressed_size = frame.get_frame_bytes();

    if (do_zstd_compress) {
        if (do_quantize) {
            compressed_size = zstd_compressed.size();
            compression_ratio = uncompressed_size / (float)compressed_size;
        } else {
            compressed_size = zstd_compressed.size();
            compression_ratio = uncompressed_size / (float)compressed_size;
        }
    } else {
        if (do_quantize) {
            compressed_size = compressed.size();
            compression_ratio = uncompressed_size / (float)compressed_size;
        } else {
            compressed_size = compressed.size();
            compression_ratio = uncompressed_size / (float)compressed_size;
        }
    }

    // stats.total_compress = stats.quantize_time + stats.zstd_compress_time + stats.rvl_compress_time;
    // stats.total_decompress = stats.dequantize_time + stats.zstd_decompress_time + stats.rvl_decompress_time;
    // stats.total = stats.total_compress + stats.total_compress;
    stats.uncompressed_size = uncompressed_size;
    stats.compressed_size = compressed_size;
    stats.compression_ratio = compression_ratio;
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
    DepthFrame & frame,
    Algorithm compression_algorithm,
    zdepth::DepthCompressor & comp,
    zdepth::DepthCompressor & decomp)
{
    switch (compression_algorithm)
    {
    case Algorithm::RVL:
        RVL_compress(frame, false, false);
        break;

    case Algorithm::RVL_QUANTIZE:
        RVL_compress(frame, true, false);
        break;

    case Algorithm::RVL_ZSTD:
        RVL_compress(frame, false, true);
        break;

    case Algorithm::RVL_QUANTIZE_ZSTD:
        RVL_compress(frame, true, true);
        break;

    case Algorithm::ZDEPTH_KEY_FRAME:
        Zdepth_compress(frame, true, comp, decomp);
        break;

    case Algorithm::ZDEPTH_NO_KEY_FRAME:
        Zdepth_compress(frame, false, comp, decomp);
        break;

    default: break;
    }

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


std::vector<DepthFrame>
divide_frame_into_chunks(
    DepthFrame & frame,
    size_t n_chunks)
{
    std::vector<DepthFrame> chunks;

    size_t total_pixels = frame.get_frame_pixels();
    size_t chunk_pixels = total_pixels / n_chunks;

    if (total_pixels % n_chunks) {
        cout << "Number <n_pixels> (" << total_pixels << ") not divisible by "
             << n_chunks << " <n_chunks> ." << endl;
        assert(0);
    }

    for (size_t i = 0; i < n_chunks; i++) {
        size_t offset = chunk_pixels * i;
        uint16_t * chunk_ptr = frame.data_ + offset;
        size_t chunk_height = frame.height_ / n_chunks;
        std::string chunk_name = frame.name_ + "_" + std::to_string(i);
        chunks.emplace_back(chunk_name, chunk_ptr, chunk_height, frame.width_);
    }

    return chunks;
}

void multithread_compress(
    DepthFrame & frame,
    size_t number_of_threads,
    Algorithm algo)
{
    auto chunks = divide_frame_into_chunks(frame, number_of_threads);

    std::vector<zdepth::DepthCompressor> comp;
    std::vector<zdepth::DepthCompressor> decomp;

    for (size_t i=0; i< number_of_threads; i++) {
        comp.emplace_back(zdepth::DepthCompressor());
        decomp.emplace_back(zdepth::DepthCompressor());
    }

    uint64_t start = GetTimeUsec();

    // Start compression
    std::thread t0([&]() {CompressFrame(chunks[0], algo, comp[0], decomp[0]);});
    std::thread t1([&]() {CompressFrame(chunks[1], algo, comp[1], decomp[1]);});
    std::thread t2([&]() {CompressFrame(chunks[2], algo, comp[2], decomp[2]);});
    std::thread t3([&]() {CompressFrame(chunks[3], algo, comp[3], decomp[3]);});
    std::thread t4([&]() {CompressFrame(chunks[4], algo, comp[4], decomp[4]);});
    std::thread t5([&]() {CompressFrame(chunks[5], algo, comp[5], decomp[5]);});
    std::thread t6([&]() {CompressFrame(chunks[6], algo, comp[6], decomp[6]);});
    std::thread t7([&]() {CompressFrame(chunks[7], algo, comp[7], decomp[7]);});

    t0.join();t1.join(); t2.join(); t3.join(); t4.join(); t5.join(); t6.join(); t7.join();

    uint64_t end = GetTimeUsec();

    FrameStats total_stats;

    for (size_t i=0; i< number_of_threads; i++) {
        // print_stats(chunks[i].name_, chunks[i].stats_);
        total_stats = total_stats + chunks[i].stats_;
    }

    total_stats.total = end - start;
    total_stats.compression_ratio = total_stats.uncompressed_size / (float)total_stats.compressed_size;
    total_stats.zdepth_compress_time /= number_of_threads;
    total_stats.zdepth_decompress_time /= number_of_threads;
    total_stats.quantize_time /= number_of_threads;
    total_stats.dequantize_time /= number_of_threads;
    total_stats.rvl_compress_time /= number_of_threads;
    total_stats.rvl_decompress_time /= number_of_threads;
    total_stats.zstd_compress_time /= number_of_threads;
    total_stats.zstd_decompress_time /= number_of_threads;

    print_stats(frame.name_, total_stats);
}

int test_single_frames(Algorithm compression_algorithm, size_t number_of_threads)
{
    print_header(compression_algorithm);

    // Create frames
    DepthFrame room0("Room0", TestVector0_Room0, Height, Width);
    DepthFrame room1("Room1", TestVector0_Room1, Height, Width);
    DepthFrame ceiling0("Ceiling0", TestVector1_Ceiling0, Height, Width);
    DepthFrame ceiling1("Ceiling1", TestVector1_Ceiling1, Height, Width);
    DepthFrame person0("Person0", TestVector2_Person0, Height, Width);
    DepthFrame person1("Person1", TestVector2_Person1, Height, Width);

    // Compress
    multithread_compress(room0, number_of_threads, compression_algorithm);
    multithread_compress(room1, number_of_threads, compression_algorithm);
    multithread_compress(ceiling0, number_of_threads, compression_algorithm);
    multithread_compress(ceiling1, number_of_threads, compression_algorithm);
    multithread_compress(person0, number_of_threads, compression_algorithm);
    multithread_compress(person1, number_of_threads, compression_algorithm);

    return 0;
}

int test_image_stream(Algorithm compression_algorithm, size_t n_threads)
{
    print_header(compression_algorithm);

    // Frame specs

    DIR *dir;
    struct dirent * diread;
    vector<std::string> files;

    if ((dir = opendir("./3D-data/depth_frames_chairs/")) != nullptr) {
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
        /*
         *   Resolutions:
         *     640 x 360 - eYs3D
         *     640 x 480 - sim
         *     848 x 480 - realsense
         */

        constexpr uint16_t width = 848;
        constexpr uint16_t height = 480;
        constexpr size_t total_pixeles = width * height;
        uint16_t depth_image[total_pixeles];

        // Get frame full name
        std::string full_name = "./3D-data/depth_frames_chairs/" + entry;

        // Copy binary frame to image array
        FILE* binary_depth_image = fopen(full_name.c_str(), "rb");
        size_t ret = fread(depth_image, sizeof(uint16_t), total_pixeles, binary_depth_image);
        fclose(binary_depth_image);

        // Skip empty files
        if (!ret) {
          continue;
        }

        DepthFrame frame(entry, depth_image, height, width);

        multithread_compress(frame, n_threads, compression_algorithm);
    }
    return 0;
}



int main(int argc, char* argv[])
{
    (void)(argc);
    (void)(argv);

    constexpr size_t n_threads = 8;

    test_single_frames(Algorithm::RVL, n_threads);
    test_single_frames(Algorithm::RVL_QUANTIZE, n_threads);
    test_single_frames(Algorithm::RVL_ZSTD, n_threads);
    test_single_frames(Algorithm::RVL_QUANTIZE_ZSTD, n_threads);
    // test_single_frames(Algorithm::ZDEPTH_KEY_FRAME, n_threads);
    // test_single_frames(Algorithm::ZDEPTH_NO_KEY_FRAME, n_threads);

    test_image_stream(Algorithm::RVL, n_threads);
    test_image_stream(Algorithm::RVL_QUANTIZE, n_threads);
    test_image_stream(Algorithm::RVL_ZSTD, n_threads);
    test_image_stream(Algorithm::RVL_QUANTIZE_ZSTD, n_threads);
    // test_image_stream(Algorithm::ZDEPTH_KEY_FRAME, n_threads);
    // test_image_stream(Algorithm::ZDEPTH_NO_KEY_FRAME, n_threads);

    return 0;
}
