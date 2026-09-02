/// \file
/// Sonar 3D-15 lifecycle driver node.

#ifndef WATERLINKED_SONAR_DRIVER_WATERLINKED_SONAR_DRIVER_HPP
#define WATERLINKED_SONAR_DRIVER_WATERLINKED_SONAR_DRIVER_HPP

#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <waterlinked_sonar_driver/srv/set_range.hpp>
#include <waterlinked_sonar_driver/waterlinked_sonar_driver_parameters.hpp>
#include <waterlinkedsonar/http/client.hpp>
#include <waterlinkedsonar/http/types.hpp>
#include <waterlinkedsonar/rip/messages.hpp>
#include <waterlinkedsonar/udp/receiver.hpp>

namespace waterlinked::sonar::ros {

/// Lifecycle node that configures a Sonar 3D-15 over HTTP and publishes its
/// UDP data stream.
///
/// The library's receive thread runs the message handlers, which publish
/// directly. Everything else runs on the executor. Parameters are read once
/// per configuration, before the receive thread starts.
class WaterlinkedSonarDriver : public rclcpp_lifecycle::LifecycleNode {
 public:
  /// Creates an unconfigured node named waterlinked_sonar_driver.
  explicit WaterlinkedSonarDriver(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// Stops receiving and disables acoustics if the node is configured.
  ~WaterlinkedSonarDriver() override;

  /// Connects to the sonar, applies the parameters to it, and creates the
  /// publishers, services and diagnostics. Fails if the sonar is unreachable.
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /// Enables acoustics if requested and starts receiving.
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /// Stops receiving and disables acoustics.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /// Releases everything created by on_configure().
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /// Stops receiving, disables acoustics, and releases resources.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  /// Stops receiving, disables acoustics, and releases resources.
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  /// Sends the acoustic, UDP and IMU settings to the sonar.
  void configure_sonar();
  /// Sets the sonar's NTP server to ntp_address, or to an unreachable address
  /// if that server's time is implausible.
  void configure_ntp();
  /// Creates the services that change acoustic settings at runtime.
  void create_services();
  /// Creates the receiver and hardware diagnostic tasks and the poll timer.
  void create_diagnostics();
  /// Joins the receive thread and disables acoustics, ignoring HTTP errors.
  void stop_receiving();
  /// Releases everything created by on_configure().
  void release_resources();

  /// Publishes the range image, point cloud and camera info of one shot.
  void publish_range_image(const RangeImageView& img);
  /// Publishes a signal-strength image and its camera info; ignores other
  /// bitmap types.
  void publish_bitmap_image(const BitmapImageView& img);
  /// Publishes each sample of a batch as one Imu message.
  void publish_imu_batch(const ImuBatchView& batch);
  /// Publishes pinhole-equivalent intrinsics for an image of this geometry.
  void publish_camera_info(const std_msgs::msg::Header& header,
                           std::uint32_t width, std::uint32_t height,
                           float fov_horizontal, float fov_vertical);
  /// Header in frame_id, stamped with \p device_ns or the receipt time
  /// according to time_source.
  std_msgs::msg::Header make_header(std::int64_t device_ns) const;

  /// Queries the sonar's temperature and status into hardware_status_ through
  /// poll_client_, connecting it first if needed.
  void poll_hardware();
  /// Diagnostic task summarizing the receive thread's counters.
  void report_receiver(diagnostic_updater::DiagnosticStatusWrapper& stat);
  /// Diagnostic task reporting the last poll_hardware() result.
  void report_hardware(diagnostic_updater::DiagnosticStatusWrapper& stat);

  std::shared_ptr<waterlinked_sonar_driver::ParamListener> param_listener_;
  waterlinked_sonar_driver::Params params_;

  std::unique_ptr<SonarClient> client_;
  /// Device identity read on configuration.
  About about_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      range_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      intensity_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr
      imu_pub_;

  // Declared after the publishers its thread uses, so it is destroyed first.
  std::unique_ptr<Receiver> receiver_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_acoustics_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_high_frequency_srv_;
  rclcpp::Service<waterlinked_sonar_driver::srv::SetRange>::SharedPtr
      set_range_srv_;

  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  /// Client with a short timeout, used only by poll_hardware().
  std::unique_ptr<SonarClient> poll_client_;
  /// Result of the last poll_hardware().
  diagnostic_msgs::msg::DiagnosticStatus hardware_status_;
  /// Start of the current activation, for reporting a silent stream.
  std::chrono::steady_clock::time_point activated_at_;
};

}  // namespace waterlinked::sonar::ros

#endif  // WATERLINKED_SONAR_DRIVER_WATERLINKED_SONAR_DRIVER_HPP
