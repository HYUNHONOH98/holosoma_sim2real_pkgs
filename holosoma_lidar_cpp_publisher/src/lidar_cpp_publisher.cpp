#include <zmq.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#ifdef HOLOSOMA_HAS_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#endif

namespace {

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::uint8_t kMsgTypeLidar = 3;
constexpr std::uint8_t kFormatPointCloud2 = 0;
constexpr std::uint8_t kFormatLivoxCustom = 1;

#pragma pack(push, 1)
struct LidarFrameHeader {
  std::uint8_t version;
  std::uint8_t msg_type;
  std::uint8_t message_format;
  std::uint8_t reserved;
  std::uint32_t point_count;
  std::uint32_t sequence;
  std::uint64_t timebase_ns;
  std::uint32_t scan_period_ns;
  float min_range;
  float publish_hz;
  std::uint32_t scan_lines;
  std::uint32_t max_points;
};
#pragma pack(pop)

static_assert(sizeof(LidarFrameHeader) == 40, "Unexpected LiDAR header size");

struct PointView {
  float x;
  float y;
  float z;
};

static_assert(sizeof(PointView) == 12, "Unexpected point payload size");

bool is_valid_point(const PointView & point, float min_range)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  const float range_sq = point.x * point.x + point.y * point.y + point.z * point.z;
  return range_sq > min_range * min_range;
}

std::size_t effective_stride(std::size_t point_count, std::uint32_t max_points)
{
  if (max_points == 0U || point_count <= static_cast<std::size_t>(max_points)) {
    return 1U;
  }
  return static_cast<std::size_t>(
    std::ceil(static_cast<double>(point_count) / static_cast<double>(max_points)));
}

std::string format_name(std::uint8_t format)
{
  if (format == kFormatLivoxCustom) {
    return "livox_custom";
  }
  return "pointcloud2";
}

}  // namespace

class LidarCppPublisher final : public rclcpp::Node {
 public:
  LidarCppPublisher()
  : Node("holosoma_lidar_cpp_publisher")
  {
    endpoint_ = declare_parameter<std::string>("endpoint", "tcp://127.0.0.1:5557");
    topic_ = declare_parameter<std::string>("topic", "/livox/lidar");
    frame_id_fallback_ = declare_parameter<std::string>("frame_id", "mid360_link_frame");
    message_type_ = declare_parameter<std::string>("message_type", "pointcloud2");
    receive_timeout_ms_ = declare_parameter<int>("receive_timeout_ms", 100);
    rcv_hwm_ = declare_parameter<int>("receive_hwm", 1);
    use_conflate_ = declare_parameter<bool>("conflate", false);

    std::transform(message_type_.begin(), message_type_.end(), message_type_.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

    if (message_type_ == "pointcloud2" || message_type_ == "sensor_msgs/msg/pointcloud2") {
      output_format_ = kFormatPointCloud2;
      pointcloud2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, rclcpp::SensorDataQoS());
    } else if (
      message_type_ == "livox" || message_type_ == "livox_custom" || message_type_ == "custom_msg" ||
      message_type_ == "livox_ros_driver2/msg/custommsg") {
      output_format_ = kFormatLivoxCustom;
#ifdef HOLOSOMA_HAS_LIVOX_ROS_DRIVER2
      livox_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(topic_, rclcpp::SensorDataQoS());
#else
      throw std::runtime_error(
        "message_type=livox_custom requires livox_ros_driver2 to be available when this package is built.");
#endif
    } else {
      throw std::runtime_error("message_type must be 'pointcloud2' or 'livox_custom'.");
    }

    init_zmq();
    worker_ = std::thread([this]() { receive_loop(); });

    RCLCPP_INFO(
      get_logger(),
      "C++ LiDAR publisher ready: endpoint='%s', topic='%s', type='%s', conflate=%s.",
      endpoint_.c_str(),
      topic_.c_str(),
      format_name(output_format_).c_str(),
      use_conflate_ ? "true" : "false");
  }

  ~LidarCppPublisher() override
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (socket_ != nullptr) {
      zmq_close(socket_);
      socket_ = nullptr;
    }
    if (context_ != nullptr) {
      zmq_ctx_term(context_);
      context_ = nullptr;
    }
  }

 private:
  void init_zmq()
  {
    context_ = zmq_ctx_new();
    if (context_ == nullptr) {
      throw std::runtime_error("Failed to create ZeroMQ context.");
    }

    socket_ = zmq_socket(context_, ZMQ_SUB);
    if (socket_ == nullptr) {
      throw std::runtime_error("Failed to create ZeroMQ SUB socket.");
    }

    const int timeout = receive_timeout_ms_;
    const int hwm = rcv_hwm_;
    const int conflate = use_conflate_ ? 1 : 0;
    zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(socket_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);
    if (use_conflate_) {
      zmq_setsockopt(socket_, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    }

    if (zmq_connect(socket_, endpoint_.c_str()) != 0) {
      throw std::runtime_error("Failed to connect ZeroMQ SUB socket to " + endpoint_);
    }
  }

  bool recv_part(std::vector<std::uint8_t> & out)
  {
    zmq_msg_t msg;
    zmq_msg_init(&msg);
    const int rc = zmq_msg_recv(&msg, socket_, 0);
    if (rc < 0) {
      zmq_msg_close(&msg);
      return false;
    }

    const auto size = static_cast<std::size_t>(zmq_msg_size(&msg));
    const auto * data = static_cast<const std::uint8_t *>(zmq_msg_data(&msg));
    out.assign(data, data + size);
    zmq_msg_close(&msg);
    return true;
  }

  void receive_loop()
  {
    std::vector<std::uint8_t> header_bytes;
    std::vector<std::uint8_t> frame_bytes;
    std::vector<std::uint8_t> payload_bytes;

    while (rclcpp::ok() && running_.load()) {
      if (!recv_part(header_bytes)) {
        continue;
      }
      if (!recv_part(frame_bytes) || !recv_part(payload_bytes)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropped malformed multipart LiDAR ZMQ frame.");
        continue;
      }
      handle_frame(header_bytes, frame_bytes, payload_bytes);
    }
  }

  void handle_frame(
    const std::vector<std::uint8_t> & header_bytes,
    const std::vector<std::uint8_t> & frame_bytes,
    const std::vector<std::uint8_t> & payload_bytes)
  {
    if (header_bytes.size() != sizeof(LidarFrameHeader)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropped LiDAR frame with invalid header size.");
      return;
    }

    LidarFrameHeader header{};
    std::memcpy(&header, header_bytes.data(), sizeof(header));
    if (header.version != kProtocolVersion || header.msg_type != kMsgTypeLidar) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropped LiDAR frame with unsupported protocol.");
      return;
    }

    const std::size_t expected_bytes = static_cast<std::size_t>(header.point_count) * 3U * sizeof(float);
    if (payload_bytes.size() < expected_bytes) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropped truncated LiDAR point payload.");
      return;
    }
    if (header.message_format != output_format_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "LiDAR frame format '%s' differs from node output format '%s'; publishing as node output format.",
        format_name(header.message_format).c_str(),
        format_name(output_format_).c_str());
    }

    std::string frame_id(frame_bytes.begin(), frame_bytes.end());
    if (frame_id.empty()) {
      frame_id = frame_id_fallback_;
    }

    std::vector<PointView> points(static_cast<std::size_t>(header.point_count));
    if (!points.empty()) {
      std::memcpy(points.data(), payload_bytes.data(), expected_bytes);
    }

    const auto now = get_clock()->now();
    if (output_format_ == kFormatPointCloud2) {
      publish_pointcloud2(points.data(), header, frame_id, now);
    } else {
      publish_livox(points.data(), header, frame_id, now);
    }
  }

  void publish_pointcloud2(
    const PointView * points,
    const LidarFrameHeader & header,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.point_step = 16;
    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;

    const auto raw_count = static_cast<std::size_t>(header.point_count);
    const auto stride = effective_stride(raw_count, header.max_points);
    msg.data.reserve((raw_count / stride + 1U) * msg.point_step);

    const float min_range = std::max(header.min_range, 0.0F);
    const float intensity = 1.0F;
    for (std::size_t i = 0; i < raw_count; i += stride) {
      const PointView & p = points[i];
      if (!is_valid_point(p, min_range)) {
        continue;
      }
      const std::uint8_t * raw = reinterpret_cast<const std::uint8_t *>(&p.x);
      msg.data.insert(msg.data.end(), raw, raw + 3U * sizeof(float));
      const std::uint8_t * intensity_raw = reinterpret_cast<const std::uint8_t *>(&intensity);
      msg.data.insert(msg.data.end(), intensity_raw, intensity_raw + sizeof(float));
    }

    msg.width = static_cast<std::uint32_t>(msg.data.size() / msg.point_step);
    msg.row_step = msg.point_step * msg.width;
    pointcloud2_pub_->publish(msg);
    log_publish(msg.width, frame_id);
  }

  void publish_livox(
    const PointView * points,
    const LidarFrameHeader & header,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
  {
#ifdef HOLOSOMA_HAS_LIVOX_ROS_DRIVER2
    livox_ros_driver2::msg::CustomMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.lidar_id = 0;
    msg.rsvd = {0, 0, 0};
    msg.timebase = header.timebase_ns != 0U ? header.timebase_ns : static_cast<std::uint64_t>(stamp.nanoseconds());

    const auto raw_count = static_cast<std::size_t>(header.point_count);
    const auto stride = effective_stride(raw_count, header.max_points);
    msg.points.reserve(raw_count / stride + 1U);

    const float min_range = std::max(header.min_range, 0.0F);
    const std::uint32_t scan_period_ns =
      header.scan_period_ns != 0U ? header.scan_period_ns : static_cast<std::uint32_t>(100000000U);
    const std::uint32_t scan_lines = std::max(header.scan_lines, 1U);

    for (std::size_t i = 0; i < raw_count; i += stride) {
      const PointView & p = points[i];
      if (!is_valid_point(p, min_range)) {
        continue;
      }
      livox_ros_driver2::msg::CustomPoint point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      point.reflectivity = 100U;
      point.tag = 0x10U;
      point.line = static_cast<std::uint8_t>(msg.points.size() % scan_lines);
      msg.points.push_back(point);
    }

    msg.point_num = static_cast<std::uint32_t>(msg.points.size());
    const std::uint32_t denom = std::max(msg.point_num, 1U) - 1U;
    for (std::uint32_t i = 0; i < msg.point_num; ++i) {
      msg.points[i].offset_time = denom == 0U ? 0U : static_cast<std::uint32_t>(
        std::llround(static_cast<double>(i) * static_cast<double>(scan_period_ns) / static_cast<double>(denom)));
    }

    livox_pub_->publish(msg);
    log_publish(msg.point_num, frame_id);
#else
    (void)points;
    (void)header;
    (void)frame_id;
    (void)stamp;
#endif
  }

  void log_publish(std::uint32_t width, const std::string & frame_id)
  {
    const auto count = ++publish_count_;
    if (count == 1U || count % 200U == 0U) {
      RCLCPP_INFO(
        get_logger(),
        "Published LiDAR frame #%zu: topic='%s', points=%u, frame_id='%s'.",
        count,
        topic_.c_str(),
        width,
        frame_id.c_str());
    }
  }

  std::string endpoint_;
  std::string topic_;
  std::string frame_id_fallback_;
  std::string message_type_;
  std::uint8_t output_format_{kFormatPointCloud2};
  int receive_timeout_ms_{100};
  int rcv_hwm_{1};
  bool use_conflate_{true};

  void * context_{nullptr};
  void * socket_{nullptr};
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::atomic<std::size_t> publish_count_{0U};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;
#ifdef HOLOSOMA_HAS_LIVOX_ROS_DRIVER2
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pub_;
#endif
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LidarCppPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("holosoma_lidar_cpp_publisher"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
