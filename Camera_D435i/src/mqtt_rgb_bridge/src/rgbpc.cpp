#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2-gl/rs_processing_gl.hpp>

#include <opencv2/opencv.hpp>
#include <zlib.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// ------------------------- binary helpers -------------------------
static inline void append_u32(std::vector<uint8_t> &b, uint32_t v) {
  uint8_t tmp[4]; std::memcpy(tmp, &v, 4); b.insert(b.end(), tmp, tmp + 4);
}
static inline void append_u64(std::vector<uint8_t> &b, uint64_t v) {
  uint8_t tmp[8]; std::memcpy(tmp, &v, 8); b.insert(b.end(), tmp, tmp + 8);
}
static inline void append_f32(std::vector<uint8_t> &b, float v) {
  uint8_t tmp[4]; std::memcpy(tmp, &v, 4); b.insert(b.end(), tmp, tmp + 4);
}
static inline void append_bytes(std::vector<uint8_t> &b, const uint8_t *p, size_t n) {
  b.insert(b.end(), p, p + n);
}
static inline void write_u32_at(std::vector<uint8_t> &b, size_t off, uint32_t v) {
  std::memcpy(b.data() + off, &v, 4);
}
static bool zlib_compress(const uint8_t *in, size_t in_sz, std::vector<uint8_t> &out, int level = Z_BEST_SPEED) {
  uLongf bound = compressBound(static_cast<uLong>(in_sz));
  out.resize(bound);
  int rc = compress2(out.data(), &bound, in, static_cast<uLong>(in_sz), level);
  if (rc != Z_OK) return false;
  out.resize(bound);
  return true;
}

class RealSenseGpuCombinedNode : public rclcpp::Node {
public:
  RealSenseGpuCombinedNode() : Node("realsense_gpu_combined") {
    declare_parameter<int>("width", 640);      // RealSense standard
    declare_parameter<int>("height", 360);
    declare_parameter<int>("fps", 30);
    declare_parameter<int>("jpeg_quality", 50);
    declare_parameter<int>("pc_step", 15);     // bigger => lighter stream
    declare_parameter<double>("min_dist", 0.05);
    declare_parameter<double>("max_dist", 2.5);

    // IMPORTANT: "as camera sees them" => keep camera frame by default
    // (RealSense pointcloud frame: x right, y down, z forward)
    declare_parameter<bool>("axis_to_robot", false);
    declare_parameter<std::string>("rgb_topic", "/camera/rgb/image_compressed");
    declare_parameter<std::string>("combined_topic", "/camera/combined");
    declare_parameter<double>("z_offset", 0.0);
    z_offset_ = get_parameter("z_offset").as_double();

    width_  = (int)get_parameter("width").as_int();
    height_ = (int)get_parameter("height").as_int();
    fps_    = std::max(1, (int)get_parameter("fps").as_int());
    jpeg_quality_ = (int)get_parameter("jpeg_quality").as_int();
    pc_step_ = std::max(1, (int)get_parameter("pc_step").as_int());
    min_dist_ = get_parameter("min_dist").as_double();
    max_dist_ = get_parameter("max_dist").as_double();
    axis_to_robot_ = get_parameter("axis_to_robot").as_bool();
    rgb_topic_ = get_parameter("rgb_topic").as_string();
    combined_topic_ = get_parameter("combined_topic").as_string();

    rgb_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(rgb_topic_, rclcpp::SensorDataQoS());
    combined_pub_ = create_publisher<std_msgs::msg::ByteMultiArray>(combined_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "✅ Node started. Topics:\n  %s\n  %s\nParams: %dx%d @ %dfps, jpeg_q=%d, pc_step=%d axis_to_robot=%d",
      rgb_topic_.c_str(), combined_topic_.c_str(),
      width_, height_, fps_, jpeg_quality_, pc_step_, axis_to_robot_ ? 1 : 0);

    // Setup depth filters (kept)
    threshold_filter_ = std::make_unique<rs2::threshold_filter>((float)min_dist_, (float)max_dist_);
    decimation_filter_ = std::make_unique<rs2::decimation_filter>();
    depth2disp_ = std::make_unique<rs2::disparity_transform>();
     disp2depth_ = std::make_unique<rs2::disparity_transform>(false);
    spatial_filter_ = std::make_unique<rs2::spatial_filter>();
    temporal_filter_ = std::make_unique<rs2::temporal_filter>();

    decimation_filter_->set_option(RS2_OPTION_FILTER_MAGNITUDE, 5);
    temporal_filter_->set_option(RS2_OPTION_HOLES_FILL, 0);

    // Start pipeline WITHOUT IMU streams
    start_pipeline_with_fallback();

    // Align depth to color
    align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);

    // GPU objects (kept)
    gpu_available_ = false;
    try {
      rs2::gl::init_rendering();
      gl_pc_ = std::make_unique<rs2::gl::pointcloud>();
      uploader_ = std::make_unique<rs2::gl::uploader>();
      gpu_available_ = true;
      RCLCPP_INFO(get_logger(), "GPU available: YES");
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "GPU init failed -> CPU fallback only. (%s)", e.what());
      gpu_available_ = false;
    }

    // Warmup
    for (int i = 0; i < 20; ++i) pipe_.wait_for_frames();
    int period_ms = (int)(1000.0 / std::max(1, fps_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                              std::bind(&RealSenseGpuCombinedNode::tick, this));
  }

  ~RealSenseGpuCombinedNode() override {
    if (gpu_available_) {
      rs2::gl::shutdown_rendering();
    }
  }

private:
  void start_pipeline_with_fallback() {
    struct Res { int w; int h; };
    std::vector<Res> tries = {
      {width_, height_},
      {424, 240},
      {640, 480}
    };

    bool ok = false;
    std::string last_err;

    for (auto &r : tries) {
      try {
        rs2::config c;
        c.enable_stream(RS2_STREAM_DEPTH, r.w, r.h, RS2_FORMAT_Z16, fps_);
        c.enable_stream(RS2_STREAM_COLOR, r.w, r.h, RS2_FORMAT_RGB8, fps_);
        profile_ = pipe_.start(c);
        width_ = r.w;
        height_ = r.h;
        ok = true;
        RCLCPP_INFO(get_logger(), "RealSense started at %dx%d", width_, height_);
        break;
      } catch (const rs2::error &e) {
        last_err = e.what();
      }
    }

    if (!ok) {
      throw std::runtime_error("Failed to start RealSense pipeline. Last error: " + last_err);
    }
  }

  std::vector<uint8_t> encode_jpeg(const rs2::video_frame &color) {
    int w = color.get_width();
    int h = color.get_height();
    cv::Mat rgb(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", bgr, buf, params)) return {};
    return std::vector<uint8_t>(buf.begin(), buf.end());
  }
  // Pack XYZ + UV (NO IMU compensation anymore)
  std::vector<uint8_t> pack_pointcloud_raw(const rs2::points &points, int step) {
    const rs2::vertex *v = points.get_vertices();
    const rs2::texture_coordinate *uv = points.get_texture_coordinates();
    const int s = (int)points.size();
    if (!v || !uv || s <= 0) return {};

    std::vector<uint8_t> raw;
    raw.reserve(4 + (s / std::max(1, step)) * 5 * 4);

    append_u32(raw, 0); // placeholder count
    uint32_t count = 0;

    for (int i = 0; i < s; i += step) {
      float x = v[i].x, y = v[i].y, z = v[i].z;

      // reject invalid
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (z <= 0.0f) continue;
      if (z < (float)min_dist_ || z > (float)max_dist_) continue;

      // optional axis mapping (default FALSE to keep camera frame)
      if (axis_to_robot_) {
        // RealSense (x right, y down, z forward) -> Robotics (X forward, Y left, Z up)
        float Xw = z;
        float Yw = -x;
        float Zw = -y;
        x = Xw; y = Yw; z = Zw;
      } 
      append_f32(raw, x);
      append_f32(raw, y);
      append_f32(raw, z);
      append_f32(raw, uv[i].u);
      append_f32(raw, uv[i].v);
      count++;
    }

    write_u32_at(raw, 0, count);
    return raw;
  }

  rs2::points make_pointcloud_gpu(const rs2::video_frame &color, const rs2::depth_frame &depth) {
    gl_pc_->map_to(color);
    rs2::frame depth_gpu = uploader_->process(depth);
    return gl_pc_->calculate(depth_gpu);
  }

  rs2::points make_pointcloud_cpu(const rs2::video_frame &color, const rs2::depth_frame &depth) {
    rs2::pointcloud pc;
    pc.map_to(color);
    return pc.calculate(depth);
  }

  void tick() {
    rs2::frameset fs;
    if (!pipe_.poll_for_frames(&fs)) return;
    // Align depth to color
    rs2::frameset aligned = align_to_color_->process(fs);
    rs2::video_frame color = aligned.get_color_frame();
    rs2::depth_frame depth = aligned.get_depth_frame();
    if (!color || !depth) return;

    // Publish RGB as compressed image
    std::vector<uint8_t> jpeg = encode_jpeg(color);
    if (!jpeg.empty()) {
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp = now();
      msg.header.frame_id = "camera_color_frame";
      msg.format = "jpeg";
      msg.data = jpeg;
      rgb_pub_->publish(msg);
    }

    // Depth filters (kept)
    depth = depth.apply_filter(*threshold_filter_);
    depth = depth.apply_filter(*decimation_filter_);
    depth = depth.apply_filter(*depth2disp_);
    depth = depth.apply_filter(*spatial_filter_);
    depth = depth.apply_filter(*temporal_filter_);
    depth = depth.apply_filter(*disp2depth_);

    // PointCloud (GPU try -> CPU fallback)
    rs2::points pts;
    bool used_gpu = false;
    if (gpu_available_) {
      try {
        pts = make_pointcloud_gpu(color, depth);
        used_gpu = true;
      } catch (...) {
        used_gpu = false;
      }
    }
    if (!used_gpu) {
      pts = make_pointcloud_cpu(color, depth);
    }

    // Pack + compress pointcloud
    std::vector<uint8_t> pc_raw = pack_pointcloud_raw(pts, pc_step_);
    std::vector<uint8_t> pc_z;
    if (!pc_raw.empty()) {
      if (!zlib_compress(pc_raw.data(), pc_raw.size(), pc_z, Z_BEST_SPEED)) pc_z.clear();
    }

    // Combined packet (version=2 kept)
    // magic[4] + ver(u32) + stamp(u64) + w(u32) + h(u32) + yaw(f32) + pitch(f32) + roll(f32) + jpeg_sz(u32) + pc_sz(u32) + payloads
    // IMU removed => yaw/pitch/roll = 0
    std::vector<uint8_t> packet;
    packet.reserve(64 + jpeg.size() + pc_z.size());

    const uint8_t magic[4] = {'R','C','M','B'};
    append_bytes(packet, magic, 4);
    append_u32(packet, 2);
    uint64_t stamp_ns = (uint64_t)now().nanoseconds();
    append_u64(packet, stamp_ns);

    append_u32(packet, (uint32_t)width_);
    append_u32(packet, (uint32_t)height_);

    append_f32(packet, 0.0f); // yaw_deg
    append_f32(packet, 0.0f); // pitch_deg
    append_f32(packet, 0.0f); // roll_deg

    append_u32(packet, (uint32_t)jpeg.size());
    append_u32(packet, (uint32_t)pc_z.size());

    if (!jpeg.empty()) append_bytes(packet, jpeg.data(), jpeg.size());
    if (!pc_z.empty()) append_bytes(packet, pc_z.data(), pc_z.size());

    std_msgs::msg::ByteMultiArray out;
    out.data = std::move(packet);
    combined_pub_->publish(out);

    static int c = 0;
    if ((++c % (fps_ * 3)) == 0) {
      RCLCPP_INFO(get_logger(), "PointCloud path: %s (axis_to_robot=%d, pc_raw=%zu, pc_z=%zu)",
                  used_gpu ? "GPU" : "CPU", axis_to_robot_ ? 1 : 0, pc_raw.size(), pc_z.size());
    }
  }

private:
int width_{424}, height_{240}, fps_{6};
  int jpeg_quality_{50}, pc_step_{20};
  double min_dist_{0.05}, max_dist_{2.5};
  bool axis_to_robot_{false};
  double z_offset_{0.0};

  std::string rgb_topic_{"/camera/rgb/image_compressed"};
  std::string combined_topic_{"/camera/combined"};

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr combined_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rs2::pipeline pipe_;
  rs2::pipeline_profile profile_;
  std::unique_ptr<rs2::align> align_to_color_;

  bool gpu_available_{false};
  std::unique_ptr<rs2::gl::pointcloud> gl_pc_;
  std::unique_ptr<rs2::gl::uploader> uploader_;

  std::unique_ptr<rs2::threshold_filter> threshold_filter_;
  std::unique_ptr<rs2::decimation_filter> decimation_filter_;
  std::unique_ptr<rs2::disparity_transform> depth2disp_;
  std::unique_ptr<rs2::disparity_transform> disp2depth_;
  std::unique_ptr<rs2::spatial_filter> spatial_filter_;
  std::unique_ptr<rs2::temporal_filter> temporal_filter_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSenseGpuCombinedNode>());
  rclcpp::shutdown();
  return 0;
}

