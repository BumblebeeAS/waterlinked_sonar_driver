/// \file
/// Lifecycle node implementation for the Sonar 3D-15 driver.

#include "waterlinked_sonar_driver.hpp"

#include <cmath>
#include <cstring>
#include <ctime>
#include <exception>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
#include <waterlinkedsonar/http/errors.hpp>
#include <waterlinkedsonar/ntp/sntp.hpp>
#include <waterlinkedsonar/rip/conversions.hpp>
#include <waterlinkedsonar/udp/socket.hpp>

namespace waterlinked::sonar::ros {

namespace {

using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr double RADIANS_PER_DEGREE = M_PI / 180.0;
constexpr double SILENCE_WARNING_SECONDS = 5.0;
// Short, so an unresponsive sonar cannot stall the executor for long.
constexpr std::chrono::seconds POLL_TIMEOUT{1};
constexpr std::time_t PLAUSIBLE_NTP_EPOCH = 1704067200;  // 2024-01-01T00:00Z
// RFC 5737 TEST-NET-1: reserved and never routed.
constexpr const char* UNREACHABLE_NTP_ADDRESS = "192.0.2.1";

/// Runs \p apply, logging a warning instead when the firmware is too old.
template <typename Function>
void apply_if_supported(const rclcpp::Logger& logger, Function&& apply) {
  try {
    std::forward<Function>(apply)();
  } catch (const VersionError& e) {
    RCLCPP_WARN(logger, "Skipped: %s", e.what());
  }
}

/// Fills a service response from \p action, which returns the success
/// message; an exception becomes a failed response.
template <typename Response, typename Function>
void respond(const rclcpp::Logger& logger, Response& response,
             Function&& action) {
  try {
    response.message = std::forward<Function>(action)();
    response.success = true;
    RCLCPP_INFO(logger, "%s", response.message.c_str());
  } catch (const std::exception& e) {
    response.message = e.what();
    response.success = false;
  }
}

/// Throws std::invalid_argument for combinations the parameter file cannot
/// validate on its own.
void validate(const waterlinked_sonar_driver::Params& params) {
  if (params.range_max <= params.range_min) {
    throw std::invalid_argument("range_max must exceed range_min");
  }
  if (params.udp.mode == "unicast" &&
      params.udp.unicast_destination_ip.empty()) {
    throw std::invalid_argument(
        "udp.unicast_destination_ip is required in unicast mode");
  }
}

/// Whether \p server answers an SNTP query with a time from 2024 onward.
bool ntp_server_plausible(const std::string& server) {
  const auto time = sntp_query(server, std::chrono::seconds(2));
  return time &&
         *time >= std::chrono::system_clock::from_time_t(PLAUSIBLE_NTP_EPOCH);
}

}  // namespace

WaterlinkedSonarDriver::WaterlinkedSonarDriver(
    const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("waterlinked_sonar_driver", options) {}

WaterlinkedSonarDriver::~WaterlinkedSonarDriver() { stop_receiving(); }

WaterlinkedSonarDriver::CallbackReturn WaterlinkedSonarDriver::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  try {
    param_listener_ = std::make_shared<waterlinked_sonar_driver::ParamListener>(
        get_node_parameters_interface());
    params_ = param_listener_->get_params();
    validate(params_);
    client_ = std::make_unique<SonarClient>(
        params_.ip_address, static_cast<std::uint16_t>(params_.http_port),
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(params_.http_timeout)));
    about_ = client_->about();
    RCLCPP_INFO(get_logger(), "Connected to %s (chipid %s, firmware %s)",
                about_.product_name.c_str(), about_.chipid.c_str(),
                about_.version_short.c_str());
    configure_sonar();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    release_resources();
    return CallbackReturn::FAILURE;
  }

  const rclcpp::QoS qos(10);
  point_cloud_pub_ = create_publisher<PointCloud2>("~/point_cloud", qos);
  range_image_pub_ = create_publisher<Image>("~/range_image", qos);
  intensity_image_pub_ = create_publisher<Image>("~/intensity_image", qos);
  camera_info_pub_ = create_publisher<CameraInfo>("~/camera_info", qos);
  if (params_.imu_enabled) {
    imu_pub_ = create_publisher<Imu>("~/imu", qos);
  }
  create_services();
  create_diagnostics();
  return CallbackReturn::SUCCESS;
}

void WaterlinkedSonarDriver::configure_sonar() {
  client_->set_speed_of_sound(params_.speed_of_sound);
  client_->set_range({params_.range_min, params_.range_max});
  apply_if_supported(get_logger(), [this] {
    client_->set_mode(params_.mode == "high-frequency"
                          ? AcousticsMode::HIGH_FREQUENCY
                          : AcousticsMode::LOW_FREQUENCY);
  });
  apply_if_supported(get_logger(), [this] {
    client_->set_salinity(params_.salinity == "fresh" ? Salinity::FRESH
                                                      : Salinity::SALT);
  });

  UdpConfig udp;
  if (params_.udp.mode == "unicast") {
    udp.mode = UdpConfig::Mode::UNICAST;
    udp.unicast_destination_ip = params_.udp.unicast_destination_ip;
    udp.unicast_destination_port = static_cast<std::uint16_t>(params_.udp.port);
  }
  client_->set_udp_config(udp);

  apply_if_supported(get_logger(), [this] {
    client_->set_imu_batch_enabled(params_.imu_enabled);
  });
  if (!params_.ntp_address.empty()) {
    apply_if_supported(get_logger(), [this] { configure_ntp(); });
  }
  if (params_.time_source == "device") {
    apply_if_supported(get_logger(), [this] {
      if (!client_->time_status().ntp_synced) {
        RCLCPP_WARN(get_logger(), "The sonar clock is not NTP-synchronized");
      }
    });
  }
  RCLCPP_INFO(get_logger(),
              "Speed of sound %.1f m/s (0 is automatic), range [%.2f, %.2f] m, "
              "%s, %s water, UDP %s",
              params_.speed_of_sound, params_.range_min, params_.range_max,
              params_.mode.c_str(), params_.salinity.c_str(),
              params_.udp.mode.c_str());
}

void WaterlinkedSonarDriver::configure_ntp() {
  // Water Linked devices keep syncing to the configured server across reboots,
  // and a device synced to a clock near the epoch can stay unreachable until
  // power-cycled. "auto" is resolved by the device and cannot be probed.
  std::string address = params_.ntp_address;
  if (address != "auto" && !ntp_server_plausible(address)) {
    RCLCPP_WARN(get_logger(),
                "NTP server %s did not answer with a time from 2024 onward; "
                "configuring the unreachable address %s instead",
                address.c_str(), UNREACHABLE_NTP_ADDRESS);
    address = UNREACHABLE_NTP_ADDRESS;
  }
  client_->set_ntp_address(address);
  if (address == UNREACHABLE_NTP_ADDRESS) {
    return;
  }
  const ForceSyncResult result = client_->force_sync_ntp(
      std::chrono::duration<double>(params_.http_timeout));
  RCLCPP_INFO(get_logger(), "NTP server %s, sync %s: %s", address.c_str(),
              result.success ? "succeeded" : "failed", result.message.c_str());
}

void WaterlinkedSonarDriver::create_services() {
  using std_srvs::srv::SetBool;
  using waterlinked_sonar_driver::srv::SetRange;

  enable_acoustics_srv_ = create_service<SetBool>(
      "~/enable_acoustics",
      [this](const std::shared_ptr<SetBool::Request> request,
             std::shared_ptr<SetBool::Response> response) {
        respond(get_logger(), *response, [&] {
          client_->set_acoustics_enabled(request->data);
          return request->data ? "Acoustics enabled" : "Acoustics disabled";
        });
      });

  enable_high_frequency_srv_ = create_service<SetBool>(
      "~/enable_high_frequency",
      [this](const std::shared_ptr<SetBool::Request> request,
             std::shared_ptr<SetBool::Response> response) {
        respond(get_logger(), *response, [&] {
          const AcousticsMode mode = request->data
                                         ? AcousticsMode::HIGH_FREQUENCY
                                         : AcousticsMode::LOW_FREQUENCY;
          client_->set_mode(mode);
          return std::string("Mode set to ") + to_string(mode);
        });
      });

  set_range_srv_ = create_service<SetRange>(
      "~/set_range", [this](const std::shared_ptr<SetRange::Request> request,
                            std::shared_ptr<SetRange::Response> response) {
        respond(get_logger(), *response, [&] {
          if (request->min < 0.0 || request->max <= request->min) {
            throw std::invalid_argument("range must satisfy 0 <= min < max");
          }
          client_->set_range({request->min, request->max});
          return "Range set to [" + std::to_string(request->min) + ", " +
                 std::to_string(request->max) + "] m";
        });
      });
}

void WaterlinkedSonarDriver::create_diagnostics() {
  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID(params_.ip_address);
  diagnostics_->add("Sonar 3D-15 Receiver", this,
                    &WaterlinkedSonarDriver::report_receiver);
  diagnostics_->add("Sonar 3D-15", this,
                    &WaterlinkedSonarDriver::report_hardware);
  poll_hardware();
  poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(params_.diagnostics_period),
      [this] { poll_hardware(); });
}

WaterlinkedSonarDriver::CallbackReturn WaterlinkedSonarDriver::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  UdpSocketConfig socket;
  socket.mode = params_.udp.mode == "unicast"
                    ? UdpSocketConfig::Mode::UNICAST
                    : UdpSocketConfig::Mode::MULTICAST;
  socket.port = static_cast<std::uint16_t>(params_.udp.port);
  socket.interface_ip = params_.udp.interface_ip;
  ReceiverOptions options;
  if (params_.udp.filter_source_ip) {
    options.source_ip = params_.ip_address;
  }

  try {
    receiver_ = std::make_unique<Receiver>(UdpSocket(socket), options);
    client_->set_acoustics_enabled(params_.acoustics_enabled);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Activation failed: %s", e.what());
    receiver_ = nullptr;
    return CallbackReturn::FAILURE;
  }

  receiver_->on_range_image(
      [this](const RangeImageView& img) { publish_range_image(img); });
  receiver_->on_bitmap_image(
      [this](const BitmapImageView& img) { publish_bitmap_image(img); });
  if (imu_pub_) {
    receiver_->on_imu_batch(
        [this](const ImuBatchView& batch) { publish_imu_batch(batch); });
  }
  receiver_->on_socket_error([this](const std::error_code& error) {
    RCLCPP_ERROR(get_logger(), "Receiving stopped: %s",
                 error.message().c_str());
  });

  point_cloud_pub_->on_activate();
  range_image_pub_->on_activate();
  intensity_image_pub_->on_activate();
  camera_info_pub_->on_activate();
  if (imu_pub_) {
    imu_pub_->on_activate();
  }
  activated_at_ = std::chrono::steady_clock::now();
  receiver_->start();
  RCLCPP_INFO(get_logger(), "Receiving %s on UDP port %d",
              params_.udp.mode.c_str(), static_cast<int>(params_.udp.port));
  return CallbackReturn::SUCCESS;
}

WaterlinkedSonarDriver::CallbackReturn WaterlinkedSonarDriver::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  stop_receiving();
  point_cloud_pub_->on_deactivate();
  range_image_pub_->on_deactivate();
  intensity_image_pub_->on_deactivate();
  camera_info_pub_->on_deactivate();
  if (imu_pub_) {
    imu_pub_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

WaterlinkedSonarDriver::CallbackReturn WaterlinkedSonarDriver::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  release_resources();
  return CallbackReturn::SUCCESS;
}

WaterlinkedSonarDriver::CallbackReturn WaterlinkedSonarDriver::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/) {
  stop_receiving();
  release_resources();
  return CallbackReturn::SUCCESS;
}

WaterlinkedSonarDriver::CallbackReturn WaterlinkedSonarDriver::on_error(
    const rclcpp_lifecycle::State& /*state*/) {
  stop_receiving();
  release_resources();
  return CallbackReturn::SUCCESS;
}

void WaterlinkedSonarDriver::stop_receiving() {
  receiver_ = nullptr;
  if (!client_) {
    return;
  }
  try {
    client_->set_acoustics_enabled(false);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Could not disable acoustics: %s", e.what());
  }
}

void WaterlinkedSonarDriver::release_resources() {
  receiver_ = nullptr;
  poll_timer_ = nullptr;
  diagnostics_ = nullptr;
  enable_acoustics_srv_ = nullptr;
  enable_high_frequency_srv_ = nullptr;
  set_range_srv_ = nullptr;
  point_cloud_pub_ = nullptr;
  range_image_pub_ = nullptr;
  intensity_image_pub_ = nullptr;
  camera_info_pub_ = nullptr;
  imu_pub_ = nullptr;
  poll_client_ = nullptr;
  client_ = nullptr;
  param_listener_ = nullptr;
}

std_msgs::msg::Header WaterlinkedSonarDriver::make_header(
    std::int64_t device_ns) const {
  std_msgs::msg::Header header;
  header.stamp =
      params_.time_source == "device" ? rclcpp::Time(device_ns) : now();
  header.frame_id = params_.frame_id;
  return header;
}

void WaterlinkedSonarDriver::publish_range_image(const RangeImageView& img) {
  const std_msgs::msg::Header header = make_header(img.header.timestamp_ns);
  publish_camera_info(header, img.width, img.height, img.fov_horizontal,
                      img.fov_vertical);

  std::vector<float> values;
  if (!range_image_to_distances(img, values)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropped a range image with inconsistent size");
    return;
  }
  auto image = std::make_unique<Image>();
  image->header = header;
  image->width = img.width;
  image->height = img.height;
  image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image->step = img.width * sizeof(float);
  image->data.resize(values.size() * sizeof(float));
  std::memcpy(image->data.data(), values.data(), image->data.size());
  range_image_pub_->publish(std::move(image));

  if (!range_image_to_points(img, values,
                             params_.organized_cloud
                                 ? PointLayout::ORGANIZED
                                 : PointLayout::UNORGANIZED) ||
      values.empty()) {
    return;
  }
  auto cloud = std::make_unique<PointCloud2>();
  cloud->header = header;
  sensor_msgs::PointCloud2Modifier(*cloud).setPointCloud2Fields(
      3, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1,
      PointField::FLOAT32);
  if (params_.organized_cloud) {
    cloud->width = img.width;
    cloud->height = img.height;
    cloud->is_dense = false;
  } else {
    cloud->width = static_cast<std::uint32_t>(values.size() / 3);
    cloud->height = 1;
    cloud->is_dense = true;
  }
  cloud->row_step = cloud->width * cloud->point_step;
  cloud->data.resize(values.size() * sizeof(float));
  std::memcpy(cloud->data.data(), values.data(), cloud->data.size());
  point_cloud_pub_->publish(std::move(cloud));
}

void WaterlinkedSonarDriver::publish_bitmap_image(const BitmapImageView& img) {
  if (img.type != BitmapImageType::SIGNAL_STRENGTH) {
    return;
  }
  const std_msgs::msg::Header header = make_header(img.header.timestamp_ns);
  publish_camera_info(header, img.width, img.height, img.fov_horizontal,
                      img.fov_vertical);

  auto image = std::make_unique<Image>();
  image->header = header;
  image->width = img.width;
  image->height = img.height;
  image->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  image->step = img.width;
  image->data.assign(img.image_pixel_data.begin(), img.image_pixel_data.end());
  intensity_image_pub_->publish(std::move(image));
}

void WaterlinkedSonarDriver::publish_imu_batch(const ImuBatchView& batch) {
  const std::size_t samples = batch.samples;
  if (samples == 0 || batch.timestamps_ns.size() != samples ||
      batch.specific_force.size() != samples * 3 ||
      batch.rate_of_turn.size() != samples * 3) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropped an IMU batch with inconsistent size");
    return;
  }
  const bool device_time = params_.time_source == "device";
  const rclcpp::Time received = now();
  const std::int64_t newest_ns = batch.timestamps_ns[samples - 1];

  for (std::size_t i = 0; i < samples; ++i) {
    const std::int64_t sample_ns = batch.timestamps_ns[i];
    auto imu = std::make_unique<Imu>();
    imu->header.frame_id = params_.imu_frame_id;
    imu->header.stamp = device_time
                            ? rclcpp::Time(sample_ns)
                            : received - rclcpp::Duration::from_nanoseconds(
                                             newest_ns - sample_ns);
    imu->orientation_covariance[0] = -1.0;
    imu->angular_velocity.x = batch.rate_of_turn[i * 3];
    imu->angular_velocity.y = batch.rate_of_turn[(i * 3) + 1];
    imu->angular_velocity.z = batch.rate_of_turn[(i * 3) + 2];
    imu->linear_acceleration.x = batch.specific_force[i * 3];
    imu->linear_acceleration.y = batch.specific_force[(i * 3) + 1];
    imu->linear_acceleration.z = batch.specific_force[(i * 3) + 2];
    imu_pub_->publish(std::move(imu));
  }
}

void WaterlinkedSonarDriver::publish_camera_info(
    const std_msgs::msg::Header& header, std::uint32_t width,
    std::uint32_t height, float fov_horizontal, float fov_vertical) {
  const double fov_h = fov_horizontal * RADIANS_PER_DEGREE;
  const double fov_v = fov_vertical * RADIANS_PER_DEGREE;
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const double fx = fov_h > 0.0 ? cx / std::tan(fov_h / 2.0) : 0.0;
  const double fy = fov_v > 0.0 ? cy / std::tan(fov_v / 2.0) : 0.0;

  auto info = std::make_unique<CameraInfo>();
  info->header = header;
  info->width = width;
  info->height = height;
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info->d.assign(5, 0.0);
  info->k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info->p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info_pub_->publish(std::move(info));
}

void WaterlinkedSonarDriver::poll_hardware() {
  diagnostic_updater::DiagnosticStatusWrapper status;
  status.add("firmware", about_.version_short);
  status.add("chipid", about_.chipid);
  status.add("product", about_.product_name);
  try {
    if (!poll_client_) {
      poll_client_ = std::make_unique<SonarClient>(
          params_.ip_address, static_cast<std::uint16_t>(params_.http_port),
          POLL_TIMEOUT);
    }
    status.addf("temperature_c", "%.1f", poll_client_->temperature());
    const Status health = poll_client_->status();
    status.add("api_status", health.api.status);
    status.add("temperature_status", health.temperature.status);
    status.add("systems_check", health.systems_check.status);
    if (health.time) {
      status.add("time_status", health.time->status);
    }
    std::string problems;
    for (const StatusEntry* entry :
         {&health.api, &health.temperature, &health.systems_check}) {
      if (!entry->operational) {
        problems +=
            (problems.empty() ? "" : "; ") + entry->id + ": " + entry->message;
      }
    }
    if (problems.empty()) {
      status.summary(DiagnosticStatus::OK, "All systems operational");
    } else {
      status.summary(DiagnosticStatus::WARN, problems);
    }
  } catch (const VersionError&) {
    status.summary(DiagnosticStatus::OK,
                   "Status API not available on this firmware");
  } catch (const std::exception& e) {
    status.summary(DiagnosticStatus::ERROR,
                   std::string("Could not query status: ") + e.what());
  }
  hardware_status_ = status;
}

void WaterlinkedSonarDriver::report_hardware(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  stat.summary(hardware_status_);
  stat.values = hardware_status_.values;
}

void WaterlinkedSonarDriver::report_receiver(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (!receiver_) {
    stat.summary(DiagnosticStatus::OK, "Inactive");
    return;
  }
  const ReceiverStats stats = receiver_->stats();
  const auto now = std::chrono::steady_clock::now();
  const bool received_any = stats.datagrams_received > 0;
  const double silence_s =
      std::chrono::duration<double>(
          now - (received_any ? stats.last_datagram_time : activated_at_))
          .count();

  if (!receiver_->running()) {
    stat.summary(DiagnosticStatus::ERROR,
                 "Receiving stopped on a socket error; reactivate the node");
  } else if (silence_s > SILENCE_WARNING_SECONDS) {
    stat.summaryf(DiagnosticStatus::WARN, "No data for %.0f s", silence_s);
  } else if (received_any && stats.range_images == 0 &&
             stats.bitmap_images == 0 && stats.imu_batches == 0) {
    stat.summary(DiagnosticStatus::WARN,
                 "Packets received but none decoded (" +
                     std::to_string(stats.unknown_type) + " unknown)");
  } else {
    stat.summary(DiagnosticStatus::OK,
                 "Receiving (" + std::to_string(stats.range_images) +
                     " range, " + std::to_string(stats.bitmap_images) +
                     " bitmap images)");
  }
  stat.add("udp_packets_total", stats.datagrams_received);
  stat.add("range_images", stats.range_images);
  stat.add("bitmap_images", stats.bitmap_images);
  stat.add("imu_batches", stats.imu_batches);
  stat.add("unknown_packets", stats.unknown_type);
  stat.add("decode_errors", stats.decode_errors);
  stat.add("filtered_source", stats.filtered_source);
  stat.add("last_sequence_id", stats.last_sequence_id);
  stat.addf("uptime_s", "%.1f",
            std::chrono::duration<double>(now - activated_at_).count());
}

}  // namespace waterlinked::sonar::ros

RCLCPP_COMPONENTS_REGISTER_NODE(waterlinked::sonar::ros::WaterlinkedSonarDriver)
