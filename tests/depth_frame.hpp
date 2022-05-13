// Copyright: --------

using namespace std;

class FrameStats
{
public:
    FrameStats operator+(const FrameStats& r) const
    {
        FrameStats res;
        res.uncompressed_size = uncompressed_size + r.uncompressed_size;
        res.compressed_size = compressed_size + r.compressed_size;
        res.compression_ratio = compression_ratio + r.compression_ratio;
        res.quantize_time = quantize_time + r.quantize_time;
        res.dequantize_time = dequantize_time + r.dequantize_time;
        res.rvl_compress_time = rvl_compress_time + r.rvl_compress_time;
        res.rvl_decompress_time = rvl_decompress_time + r.rvl_decompress_time;
        res.zstd_compress_time = zstd_compress_time + r.zstd_compress_time;
        res.zstd_decompress_time = zstd_decompress_time + r.zstd_decompress_time;
        res.zdepth_compress_time = zdepth_compress_time + r.zdepth_compress_time;
        res.zdepth_decompress_time = zdepth_decompress_time + r.zdepth_decompress_time;
        res.total_compress = total_compress + r.total_compress;
        res.total_decompress = total_decompress + r.total_decompress;
        res.total_time = total_time + r.total_time;
        return res;
    }

    void average_time_stats(size_t n)
    {
        total_time /= n;
        zdepth_compress_time /= n;
        zdepth_decompress_time /= n;
        quantize_time /= n;
        dequantize_time /= n;
        rvl_compress_time /= n;
        rvl_decompress_time /= n;
        zstd_compress_time /= n;
        zstd_decompress_time /= n;
    }

// Stats
    size_t uncompressed_size{0};
    size_t compressed_size{0};
    float compression_ratio{0};
    uint64_t quantize_time{0};
    uint64_t dequantize_time{0};
    uint64_t rvl_compress_time{0};
    uint64_t rvl_decompress_time{0};
    uint64_t zstd_compress_time{0};
    uint64_t zstd_decompress_time{0};
    uint64_t zdepth_compress_time{0};
    uint64_t zdepth_decompress_time{0};
    uint64_t total_compress{0};
    uint64_t total_decompress{0};
    uint64_t total_time{0};
};

class DepthFrame {
public:
  DepthFrame(
    std::string name,
    uint16_t* frame,
    size_t height,
    size_t width)
    : name_(name), data_(frame), height_(height), width_(width)
  {}

  size_t get_frame_bytes() { return height_ * width_ * 2; };
  size_t get_frame_pixels() { return height_ * width_; };

  std::string name_;
  uint16_t* data_;
  size_t height_;
  size_t width_;
  FrameStats stats_;
};

void print_stats(std::string frame_name, FrameStats & stats)
{
    cout << std::left << std::setw(20) << std::setfill(' ') << frame_name;
    cout << std::left << std::setw(20) << std::setfill(' ') << stats.uncompressed_size;
    cout << std::left << std::setw(20) << std::setfill(' ') << stats.compressed_size;
    cout << std::left << std::setw(20) << std::setfill(' ') << stats.compression_ratio;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.zdepth_compress_time ;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.zdepth_decompress_time ;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.quantize_time;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.dequantize_time;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.rvl_compress_time;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.rvl_decompress_time;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.zstd_compress_time;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.zstd_decompress_time;
    cout << std::left << std::setw(15) << std::setfill(' ') << stats.total_time << endl;
}