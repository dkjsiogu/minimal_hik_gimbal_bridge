#pragma once

#include <array>
#include <algorithm>
#include <cmath>

#include <opencv2/core/types.hpp>

namespace bridge
{

enum class RightAngleRotation
{
  kIdentity,
  kRotate90CounterClockwise,
  kRotate90Clockwise,
  kRotate180,
  kUnsupported,
};

inline RightAngleRotation classify_right_angle_rotation(const std::array<double, 4> & matrix)
{
  const auto is_close = [](double lhs, double rhs) {
      return std::fabs(lhs - rhs) < 1e-9;
    };

  if (is_close(matrix[0], 1.0) && is_close(matrix[1], 0.0) &&
      is_close(matrix[2], 0.0) && is_close(matrix[3], 1.0)) {
    return RightAngleRotation::kIdentity;
  }
  if (is_close(matrix[0], 0.0) && is_close(matrix[1], 1.0) &&
      is_close(matrix[2], -1.0) && is_close(matrix[3], 0.0)) {
    return RightAngleRotation::kRotate90CounterClockwise;
  }
  if (is_close(matrix[0], 0.0) && is_close(matrix[1], -1.0) &&
      is_close(matrix[2], 1.0) && is_close(matrix[3], 0.0)) {
    return RightAngleRotation::kRotate90Clockwise;
  }
  if (is_close(matrix[0], -1.0) && is_close(matrix[1], 0.0) &&
      is_close(matrix[2], 0.0) && is_close(matrix[3], -1.0)) {
    return RightAngleRotation::kRotate180;
  }
  return RightAngleRotation::kUnsupported;
}

inline cv::Size rotated_frame_size(const cv::Size & source_size, const std::array<double, 4> & matrix)
{
  switch (classify_right_angle_rotation(matrix)) {
    case RightAngleRotation::kRotate90CounterClockwise:
    case RightAngleRotation::kRotate90Clockwise:
      return cv::Size(source_size.height, source_size.width);
    case RightAngleRotation::kIdentity:
    case RightAngleRotation::kRotate180:
    case RightAngleRotation::kUnsupported:
    default:
      return source_size;
  }
}

inline cv::Point2d map_source_point_to_rotated_frame(
  const cv::Point2d & source_point,
  const cv::Size & source_size,
  const std::array<double, 4> & matrix)
{
  switch (classify_right_angle_rotation(matrix)) {
    case RightAngleRotation::kIdentity:
      return source_point;
    case RightAngleRotation::kRotate90CounterClockwise:
      return cv::Point2d(source_point.y, static_cast<double>(source_size.width - 1) - source_point.x);
    case RightAngleRotation::kRotate90Clockwise:
      return cv::Point2d(static_cast<double>(source_size.height - 1) - source_point.y, source_point.x);
    case RightAngleRotation::kRotate180:
      return cv::Point2d(
        static_cast<double>(source_size.width - 1) - source_point.x,
        static_cast<double>(source_size.height - 1) - source_point.y);
    case RightAngleRotation::kUnsupported:
    default:
      return source_point;
  }
}

inline cv::Point2d map_rotated_point_to_source_frame(
  const cv::Point2d & rotated_point,
  const cv::Size & source_size,
  const std::array<double, 4> & matrix)
{
  switch (classify_right_angle_rotation(matrix)) {
    case RightAngleRotation::kIdentity:
      return rotated_point;
    case RightAngleRotation::kRotate90CounterClockwise:
      return cv::Point2d(
        static_cast<double>(source_size.width - 1) - rotated_point.y,
        rotated_point.x);
    case RightAngleRotation::kRotate90Clockwise:
      return cv::Point2d(
        rotated_point.y,
        static_cast<double>(source_size.height - 1) - rotated_point.x);
    case RightAngleRotation::kRotate180:
      return cv::Point2d(
        static_cast<double>(source_size.width - 1) - rotated_point.x,
        static_cast<double>(source_size.height - 1) - rotated_point.y);
    case RightAngleRotation::kUnsupported:
    default:
      return rotated_point;
  }
}

struct CropRegion
{
  cv::Rect rect{};
  double center_x = 0.5;
  double center_y = 0.5;
};

inline CropRegion compute_square_crop_region(
  const cv::Size & size,
  int requested_crop_size,
  double normalized_center_x,
  double normalized_center_y)
{
  CropRegion region;
  if (size.width <= 0 || size.height <= 0) {
    return region;
  }

  const int crop_edge = requested_crop_size > 0
    ? std::min({requested_crop_size, size.width, size.height})
    : std::min(size.width, size.height);
  const int max_x0 = std::max(0, size.width - crop_edge);
  const int max_y0 = std::max(0, size.height - crop_edge);

  const double clamped_center_x = std::clamp(normalized_center_x, 0.0, 1.0);
  const double clamped_center_y = std::clamp(normalized_center_y, 0.0, 1.0);
  const int desired_center_x = static_cast<int>(std::lround(clamped_center_x * static_cast<double>(size.width - 1)));
  const int desired_center_y = static_cast<int>(std::lround(clamped_center_y * static_cast<double>(size.height - 1)));

  const int x0 = std::clamp(desired_center_x - crop_edge / 2, 0, max_x0);
  const int y0 = std::clamp(desired_center_y - crop_edge / 2, 0, max_y0);
  region.rect = cv::Rect(x0, y0, crop_edge, crop_edge);

  if (size.width > 1) {
    region.center_x = (static_cast<double>(x0) + (static_cast<double>(crop_edge) - 1.0) * 0.5) /
      static_cast<double>(size.width - 1);
  }
  if (size.height > 1) {
    region.center_y = (static_cast<double>(y0) + (static_cast<double>(crop_edge) - 1.0) * 0.5) /
      static_cast<double>(size.height - 1);
  }
  return region;
}

}  // namespace bridge