#include "waterlinked_sonar_driver/waterlinked_sonar_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <system_error>

namespace waterlinked::sonar::ros {

namespace {

using diagnostic_msgs::msg::DiagnosticStatus;

builtin_interfaces::msg::Time to_time_msg(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  t.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return t;
}

}  // namespace

WaterlinkedSonarDriver::WaterlinkedSonarDriver(
    const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("waterlinked_sonar_driver", options) {}

CallbackReturn WaterlinkedSonarDriver::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Configuring the WaterlinkedSonarDriver");

  try {
    param_listener_ = std::make_shared<waterlinked_sonar_driver::ParamListener>(
        get_node_parameters_interface());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to get parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (params_.range_max <= params_.range_min) {
    RCLCPP_ERROR(get_logger(), "range_max (%.2f) must exceed range_min (%.2f)",
                 params_.range_max, params_.range_min);
    return CallbackReturn::ERROR;
  }
  if (params_.udp.mode == "unicast" &&
      params_.udp.unicast_destination_ip.empty()) {
    RCLCPP_ERROR(get_logger(),
                 "udp.mode is unicast but udp.unicast_destination_ip is empty");
    return CallbackReturn::ERROR;
  }

  try {
    client_ = std::make_unique<SonarClient>(
        params_.ip_address, static_cast<std::uint16_t>(params_.http_port),
        std::chrono::milliseconds(
            static_cast<std::int64_t>(params_.http_timeout * 1000.0)));
    const About about = client_->about();
    RCLCPP_INFO(get_logger(), "Connected: %s (chipid=%s, fw=%s)",
                about.product_name.c_str(), about.chipid.c_str(),
                about.version_short.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to sonar at %s:%ld: %s",
                 params_.ip_address.c_str(), params_.http_port, e.what());
    return CallbackReturn::ERROR;
  }

  if (!apply_sonar_configuration() || !configure_ntp()) {
    return CallbackReturn::ERROR;
  }

  if (params_.time_source == "device") {
    try {
      const TimeStatus ts = client_->time_status();
      if (!ts.ntp_synced) {
        RCLCPP_WARN(get_logger(),
                    "time_source is 'device' but the sonar is not NTP-synced; "
                    "stamps will drift against ROS time");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Could not check sonar time status: %s",
                  e.what());
    }
  }

  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/point_cloud", rclcpp::SensorDataQoS());
  range_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/range_image", rclcpp::SensorDataQoS());
  intensity_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/intensity_image", rclcpp::SensorDataQoS());
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/camera_info", rclcpp::SensorDataQoS());
  if (params_.imu_enabled) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu",
                                                       rclcpp::SensorDataQoS());
  }

  cloud_msg_.header.frame_id = params_.frame_id;
  cloud_msg_.fields.resize(3);
  const std::array<const char*, 3> names = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    cloud_msg_.fields[i].name = names[i];
    cloud_msg_.fields[i].offset = static_cast<std::uint32_t>(i * 4);
    cloud_msg_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg_.fields[i].count = 1;
  }
  cloud_msg_.point_step = 12;
  cloud_msg_.is_bigendian = false;

  range_msg_.header.frame_id = params_.frame_id;
  range_msg_.encoding = "32FC1";
  range_msg_.is_bigendian = 0U;

  intensity_msg_.header.frame_id = params_.frame_id;
  intensity_msg_.encoding = "8UC1";
  intensity_msg_.is_bigendian = 0U;

  camera_info_msg_.header.frame_id = params_.frame_id;
  camera_info_msg_.distortion_model = "none";

  imu_msg_.header.frame_id = params_.imu_frame_id;
  // The sonar provides no orientation estimate.
  imu_msg_.orientation_covariance[0] = -1.0;

  enable_acoustics_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_acoustics",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        try {
          client_->set_acoustics_enabled(request->data);
          response->success = true;
          response->message =
              request->data ? "Acoustics enabled" : "Acoustics disabled";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
          response->success = false;
          response->message = e.what();
        }
      });

  enable_high_frequency_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_high_frequency",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        const AcousticsMode mode = request->data ? AcousticsMode::HIGH_FREQUENCY
                                                 : AcousticsMode::LOW_FREQUENCY;
        try {
          client_->set_mode(mode);
          response->success = true;
          response->message = std::string("Mode set to ") + to_string(mode) +
                              "; expect a data blackout of several seconds";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
          response->success = false;
          response->message = e.what();
        }
      });

  set_range_srv_ = create_service<waterlinked_sonar_driver::srv::SetRange>(
      "~/set_range",
      [this](const std::shared_ptr<
                 waterlinked_sonar_driver::srv::SetRange::Request>
                 request,
             std::shared_ptr<waterlinked_sonar_driver::srv::SetRange::Response>
                 response) {
        if (request->min < 0.0 || request->max <= request->min) {
          response->success = false;
          response->message = "range must satisfy 0 <= min < max";
          return;
        }
        try {
          client_->set_range({request->min, request->max});
          response->success = true;
          response->message = "Range set to [" + std::to_string(request->min) +
                              ", " + std::to_string(request->max) + "] m";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
          response->success = false;
          response->message = e.what();
        }
      });

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID(params_.ip_address);
  diagnostics_->add("Sonar 3D-15 Receiver", this,
                    &WaterlinkedSonarDriver::receiver_diagnostics);
  diagnostics_->add("Sonar 3D-15", this,
                    &WaterlinkedSonarDriver::hardware_diagnostics);
  hardware_poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(params_.diagnostics_period),
      [this] { poll_hardware(); });

  RCLCPP_INFO(get_logger(), "WaterlinkedSonarDriver configured");
  return CallbackReturn::SUCCESS;
}

bool WaterlinkedSonarDriver::apply_sonar_configuration() {
  try {
    client_->set_speed_of_sound(params_.speed_of_sound);
    if (params_.speed_of_sound == 0.0) {
      RCLCPP_INFO(get_logger(),
                  "Speed of sound: automatic (salinity + temperature)");
    } else {
      RCLCPP_INFO(get_logger(), "Speed of sound: %.1f m/s",
                  params_.speed_of_sound);
    }

    client_->set_range({params_.range_min, params_.range_max});
    RCLCPP_INFO(get_logger(), "Range: [%.1f, %.1f] m", params_.range_min,
                params_.range_max);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to configure acoustics: %s", e.what());
    return false;
  }

  try {
    client_->set_salinity(params_.salinity == "salt" ? Salinity::SALT
                                                     : Salinity::FRESH);
    RCLCPP_INFO(get_logger(), "Salinity: %s", params_.salinity.c_str());
  } catch (const VersionError& e) {
    if (params_.salinity != "salt") {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return false;
    }
    RCLCPP_WARN(get_logger(), "Skipping salinity: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to set salinity: %s", e.what());
    return false;
  }

  try {
    client_->set_mode(params_.mode == "low-frequency"
                          ? AcousticsMode::LOW_FREQUENCY
                          : AcousticsMode::HIGH_FREQUENCY);
    RCLCPP_INFO(get_logger(), "Mode: %s", params_.mode.c_str());
  } catch (const VersionError& e) {
    if (params_.mode != "low-frequency") {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return false;
    }
    RCLCPP_WARN(get_logger(), "Skipping mode: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to set mode: %s", e.what());
    return false;
  }

  try {
    UdpConfig udp;
    if (params_.udp.mode == "multicast") {
      udp.mode = UdpConfig::Mode::MULTICAST;
      RCLCPP_INFO(get_logger(), "Sonar UDP output: multicast");
    } else {
      udp.mode = UdpConfig::Mode::UNICAST;
      udp.unicast_destination_ip = params_.udp.unicast_destination_ip;
      udp.unicast_destination_port =
          static_cast<std::uint16_t>(params_.udp.port);
      RCLCPP_INFO(get_logger(), "Sonar UDP output: unicast to %s:%ld",
                  udp.unicast_destination_ip.c_str(), params_.udp.port);
    }
    client_->set_udp_config(udp);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to configure UDP output: %s", e.what());
    return false;
  }

  try {
    client_->set_imu_batch_enabled(params_.imu_enabled);
    if (params_.imu_enabled) {
      RCLCPP_INFO(get_logger(), "IMU batch output enabled");
    }
  } catch (const VersionError& e) {
    if (params_.imu_enabled) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to configure IMU output: %s", e.what());
    return false;
  }

  return true;
}

bool WaterlinkedSonarDriver::configure_ntp() {
  if (params_.ntp_address.empty()) {
    return true;
  }

  // The sonar must never be handed an NTP server with a bad clock (e.g. this
  // host, booted with no upstream and a dead RTC, sitting near epoch). On the
  // Water Linked DVL, syncing to epoch time left the device unreachable until
  // power-cycled, with no software way back: the NTP address persists across
  // reboots, background syncs continue, and NTP cannot be disabled. Assume
  // the sonar shares that behavior. So probe the server's clock first, and
  // unless it answers with a believable time, point the sonar at an
  // unreachable address instead; a later start, once the server is sane,
  // restores it. The sonar also accepts "auto" as an address, which selects a
  // server the driver cannot know, so "auto" passes through unprobed.
  bool server_is_sane = true;
  if (params_.ntp_address != "auto") {
    const auto server_time =
        sntp_query(params_.ntp_address, std::chrono::seconds(2));
    const auto sane_epoch =
        std::chrono::system_clock::from_time_t(1704067200);  // 2024-01-01
    server_is_sane = server_time.has_value() && *server_time >= sane_epoch;
    if (!server_time) {
      RCLCPP_WARN(get_logger(),
                  "NTP server %s did not answer a time probe (down, "
                  "unreachable, or not yet synchronized)",
                  params_.ntp_address.c_str());
    } else if (!server_is_sane) {
      RCLCPP_WARN(get_logger(),
                  "NTP server %s reports a time before 2024-01-01",
                  params_.ntp_address.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(),
                "ntp_address is 'auto'; the sonar picks its own server and "
                "its clock cannot be verified before the sync");
  }

  // RFC 5737 TEST-NET-1: reserved, unroutable, never a real host.
  constexpr const char* unreachable_ntp_address = "192.0.2.1";
  const std::string ntp_address =
      server_is_sane ? params_.ntp_address : unreachable_ntp_address;
  if (!server_is_sane) {
    RCLCPP_WARN(get_logger(),
                "Pointing the sonar at an unreachable NTP address (%s) "
                "instead of %s and skipping the forced sync",
                unreachable_ntp_address, params_.ntp_address.c_str());
  }

  try {
    client_->set_ntp_address(ntp_address);
    RCLCPP_INFO(get_logger(), "Sonar NTP server: %s", ntp_address.c_str());
    if (server_is_sane) {
      const ForceSyncResult result =
          client_->force_sync_ntp(std::chrono::seconds(
              static_cast<std::int64_t>(params_.http_timeout)));
      if (result.success) {
        RCLCPP_INFO(get_logger(), "Forced NTP sync: %s",
                    result.message.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "NTP sync did not complete: %s",
                    result.message.c_str());
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to configure NTP: %s", e.what());
    return false;
  }
  return true;
}

CallbackReturn WaterlinkedSonarDriver::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    client_->set_acoustics_enabled(params_.acoustics_enabled);
    RCLCPP_INFO(get_logger(), "Acoustics %s",
                params_.acoustics_enabled ? "enabled" : "disabled");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to set acoustics: %s", e.what());
    return CallbackReturn::ERROR;
  }

  UdpSocketConfig socket_config;
  socket_config.mode = params_.udp.mode == "multicast"
                           ? UdpSocketConfig::Mode::MULTICAST
                           : UdpSocketConfig::Mode::UNICAST;
  socket_config.port = static_cast<std::uint16_t>(params_.udp.port);
  socket_config.interface_ip = params_.udp.interface_ip;

  ReceiverOptions receiver_options;
  if (params_.udp.filter_source_ip) {
    receiver_options.source_ip = params_.ip_address;
  }

  try {
    receiver_ = std::make_unique<Receiver>(
        std::make_unique<UdpSocket>(socket_config), receiver_options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to open UDP socket: %s", e.what());
    return CallbackReturn::ERROR;
  }

  receiver_->on_range_image(
      [this](const RangeImageView& img) { handle_range_image(img); });
  receiver_->on_bitmap_image(
      [this](const BitmapImageView& img) { handle_bitmap_image(img); });
  if (params_.imu_enabled) {
    receiver_->on_imu_batch(
        [this](const ImuBatchView& batch) { handle_imu_batch(batch); });
  }
  receiver_->on_socket_error([this](const std::error_code& ec) {
    RCLCPP_ERROR(get_logger(),
                 "Receive loop stopped on socket error: %s; reactivate the "
                 "node to resume",
                 ec.message().c_str());
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
  RCLCPP_INFO(get_logger(), "Receiving on %s UDP port %ld",
              params_.udp.mode.c_str(), params_.udp.port);
  return CallbackReturn::SUCCESS;
}

CallbackReturn WaterlinkedSonarDriver::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (receiver_) {
    receiver_->stop();
    receiver_ = nullptr;
  }

  point_cloud_pub_->on_deactivate();
  range_image_pub_->on_deactivate();
  intensity_image_pub_->on_deactivate();
  camera_info_pub_->on_deactivate();
  if (imu_pub_) {
    imu_pub_->on_deactivate();
  }

  // The sonar should not keep pinging with no one listening. Best-effort:
  // deactivation must succeed even if the device is unreachable.
  try {
    client_->set_acoustics_enabled(false);
    RCLCPP_INFO(get_logger(), "Acoustics disabled");
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Could not disable acoustics: %s", e.what());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn WaterlinkedSonarDriver::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  receiver_ = nullptr;
  enable_acoustics_srv_ = nullptr;
  enable_high_frequency_srv_ = nullptr;
  set_range_srv_ = nullptr;
  hardware_poll_timer_ = nullptr;
  diagnostics_ = nullptr;
  client_ = nullptr;
  return CallbackReturn::SUCCESS;
}

CallbackReturn WaterlinkedSonarDriver::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (receiver_) {
    receiver_->stop();
  }
  return on_cleanup(rclcpp_lifecycle::State());
}

builtin_interfaces::msg::Time WaterlinkedSonarDriver::stamp_for(
    std::int64_t timestamp_ns) {
  if (params_.time_source == "device") {
    return to_time_msg(timestamp_ns);
  }
  return now();
}

void WaterlinkedSonarDriver::handle_range_image(const RangeImageView& img) {
  const builtin_interfaces::msg::Time stamp =
      stamp_for(img.header.timestamp_ns);

  publish_camera_info(stamp, img.width, img.height, img.fov_horizontal,
                      img.fov_vertical);

  if (range_image_pub_->get_subscription_count() > 0) {
    if (!range_image_to_distances(img, distances_scratch_)) {
      invalid_geometry_.fetch_add(1);
    } else {
      range_msg_.header.stamp = stamp;
      range_msg_.width = img.width;
      range_msg_.height = img.height;
      range_msg_.step = img.width * 4;
      range_msg_.data.resize(distances_scratch_.size() * 4);
      std::memcpy(range_msg_.data.data(), distances_scratch_.data(),
                  range_msg_.data.size());
      range_image_pub_->publish(range_msg_);
    }
  }

  if (last_geometry_ != std::array<float, 4>{static_cast<float>(img.width),
                                             static_cast<float>(img.height),
                                             img.fov_horizontal,
                                             img.fov_vertical}) {
    last_geometry_ = {static_cast<float>(img.width),
                      static_cast<float>(img.height), img.fov_horizontal,
                      img.fov_vertical};
    RCLCPP_INFO(get_logger(), "Image geometry: %ux%u, FOV %.0fx%.0f deg",
                img.width, img.height, static_cast<double>(img.fov_horizontal),
                static_cast<double>(img.fov_vertical));
  }

  if (point_cloud_pub_->get_subscription_count() > 0) {
    if (!range_image_to_points(img, points_scratch_, params_.organized_cloud)) {
      invalid_geometry_.fetch_add(1);
      return;
    }
    const std::size_t n_points = points_scratch_.size() / 3;
    if (n_points == 0) {
      return;
    }
    cloud_msg_.header.stamp = stamp;
    if (params_.organized_cloud) {
      cloud_msg_.width = img.width;
      cloud_msg_.height = img.height;
      cloud_msg_.is_dense = false;
    } else {
      cloud_msg_.width = static_cast<std::uint32_t>(n_points);
      cloud_msg_.height = 1;
      cloud_msg_.is_dense = true;
    }
    cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step;
    cloud_msg_.data.resize(points_scratch_.size() * 4);
    std::memcpy(cloud_msg_.data.data(), points_scratch_.data(),
                cloud_msg_.data.size());
    point_cloud_pub_->publish(cloud_msg_);
  }
}

void WaterlinkedSonarDriver::handle_bitmap_image(const BitmapImageView& img) {
  if (img.type != BitmapImageType::SIGNAL_STRENGTH) {
    shaded_images_dropped_.fetch_add(1);
    return;
  }

  const builtin_interfaces::msg::Time stamp =
      stamp_for(img.header.timestamp_ns);

  publish_camera_info(stamp, img.width, img.height, img.fov_horizontal,
                      img.fov_vertical);

  if (intensity_image_pub_->get_subscription_count() > 0) {
    intensity_msg_.header.stamp = stamp;
    intensity_msg_.width = img.width;
    intensity_msg_.height = img.height;
    intensity_msg_.step = img.width;
    intensity_msg_.data.assign(img.image_pixel_data.begin(),
                               img.image_pixel_data.end());
    intensity_image_pub_->publish(intensity_msg_);
  }
}

void WaterlinkedSonarDriver::handle_imu_batch(const ImuBatchView& batch) {
  if (!imu_pub_ || imu_pub_->get_subscription_count() == 0) {
    return;
  }
  const std::size_t samples = batch.samples;
  if (batch.timestamps_ns.size() != samples ||
      batch.specific_force.size() != samples * 3 ||
      batch.rate_of_turn.size() != samples * 3) {
    invalid_geometry_.fetch_add(1);
    return;
  }

  // With time_source 'receive', per-sample stamps are the batch receipt time
  // shifted back by each sample's age within the batch.
  const std::int64_t now_ns =
      params_.time_source == "device"
          ? 0
          : static_cast<std::int64_t>(now().nanoseconds());
  const std::int64_t last_ns =
      samples > 0 ? batch.timestamps_ns[samples - 1] : 0;

  for (std::size_t i = 0; i < samples; ++i) {
    const std::int64_t sample_ns = batch.timestamps_ns[i];
    if (params_.time_source == "device") {
      imu_msg_.header.stamp = to_time_msg(sample_ns);
    } else {
      imu_msg_.header.stamp = to_time_msg(now_ns - (last_ns - sample_ns));
    }
    imu_msg_.angular_velocity.x = batch.rate_of_turn[i * 3];
    imu_msg_.angular_velocity.y = batch.rate_of_turn[(i * 3) + 1];
    imu_msg_.angular_velocity.z = batch.rate_of_turn[(i * 3) + 2];
    imu_msg_.linear_acceleration.x = batch.specific_force[i * 3];
    imu_msg_.linear_acceleration.y = batch.specific_force[(i * 3) + 1];
    imu_msg_.linear_acceleration.z = batch.specific_force[(i * 3) + 2];
    imu_pub_->publish(imu_msg_);
  }
}

void WaterlinkedSonarDriver::publish_camera_info(
    const builtin_interfaces::msg::Time& stamp, std::uint32_t width,
    std::uint32_t height, float fov_horizontal_deg, float fov_vertical_deg) {
  if (!params_.publish_camera_info ||
      camera_info_pub_->get_subscription_count() == 0) {
    return;
  }

  // Pinhole-equivalent projection so standard tools can relate pixels to
  // bearing angles. The sonar's true mapping is equiangular, so this is exact
  // only at the image center.
  const double fov_h = fov_horizontal_deg * M_PI / 180.0;
  const double fov_v = fov_vertical_deg * M_PI / 180.0;
  const double fx = fov_h > 0 ? (width / 2.0) / std::tan(fov_h / 2.0) : 0.0;
  const double fy = fov_v > 0 ? (height / 2.0) / std::tan(fov_v / 2.0) : 0.0;
  const double cx = width / 2.0;
  const double cy = height / 2.0;

  camera_info_msg_.header.stamp = stamp;
  camera_info_msg_.width = width;
  camera_info_msg_.height = height;
  camera_info_msg_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info_msg_.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info_pub_->publish(camera_info_msg_);
}

void WaterlinkedSonarDriver::receiver_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (!receiver_) {
    stat.summary(DiagnosticStatus::OK, "Inactive");
    return;
  }

  const ReceiverStats stats = receiver_->stats();
  const auto now_steady = std::chrono::steady_clock::now();
  const bool ever_received =
      stats.last_datagram_time.time_since_epoch().count() != 0;
  const double silence_s =
      std::chrono::duration<double>(now_steady - (ever_received
                                                      ? stats.last_datagram_time
                                                      : activated_at_))
          .count();

  if (!receiver_->running()) {
    stat.summary(DiagnosticStatus::ERROR,
                 "Receive thread stopped (socket error); reactivate the node");
  } else if (silence_s > 5.0) {
    stat.summary(DiagnosticStatus::WARN,
                 "No data for " + std::to_string(silence_s) + " s");
  } else if (stats.datagrams_received > 0 && stats.range_images == 0 &&
             stats.bitmap_images == 0 && stats.imu_batches == 0) {
    stat.summary(DiagnosticStatus::WARN,
                 "Datagrams arrive but none decode to known messages");
  } else {
    stat.summary(DiagnosticStatus::OK, "Receiving");
  }

  stat.add("datagrams_received", stats.datagrams_received);
  stat.add("range_images", stats.range_images);
  stat.add("bitmap_images", stats.bitmap_images);
  stat.add("imu_batches", stats.imu_batches);
  stat.add("unknown_type", stats.unknown_type);
  stat.add("decode_errors", stats.decode_errors);
  stat.add("filtered_source", stats.filtered_source);
  stat.add("shaded_images_dropped", shaded_images_dropped_.load());
  stat.add("invalid_geometry", invalid_geometry_.load());
  stat.add("last_sequence_id", stats.last_sequence_id);
}

void WaterlinkedSonarDriver::poll_hardware() {
  if (!client_) {
    return;
  }

  hardware_values_.clear();
  try {
    hardware_values_.emplace_back("temperature_c",
                                  std::to_string(client_->temperature()));
    hardware_values_.emplace_back("firmware", client_->firmware_version());

    try {
      const Status status = client_->status();
      hardware_values_.emplace_back("api_status", status.api.status);
      hardware_values_.emplace_back("temperature_status",
                                    status.temperature.status);
      hardware_values_.emplace_back("systems_check",
                                    status.systems_check.status);
      bool all_ok = status.api.operational && status.temperature.operational &&
                    status.systems_check.operational;
      std::string message = "All systems operational";
      if (!all_ok) {
        message.clear();
        for (const StatusEntry* entry :
             {&status.api, &status.temperature, &status.systems_check}) {
          if (!entry->operational) {
            message += (message.empty() ? "" : "; ") + entry->message;
          }
        }
      }
      if (status.time) {
        hardware_values_.emplace_back("time_status", status.time->status);
      }
      hardware_level_ = all_ok ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
      hardware_message_ = message;
    } catch (const VersionError&) {
      hardware_level_ = DiagnosticStatus::OK;
      hardware_message_ = "Status API not available on this firmware";
    }
  } catch (const std::exception& e) {
    hardware_level_ = DiagnosticStatus::ERROR;
    hardware_message_ = std::string("Could not query sonar: ") + e.what();
  }
}

void WaterlinkedSonarDriver::hardware_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (hardware_message_.empty()) {
    stat.summary(DiagnosticStatus::OK, "Waiting for first poll");
    return;
  }
  stat.summary(hardware_level_, hardware_message_);
  for (const auto& [key, value] : hardware_values_) {
    stat.add(key, value);
  }
}

}  // namespace waterlinked::sonar::ros

RCLCPP_COMPONENTS_REGISTER_NODE(waterlinked::sonar::ros::WaterlinkedSonarDriver)
