#include "bridge/frame.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <string>

namespace bridge
{

FrameInfo make_test_pattern_frame(uint32_t sequence, Clock::time_point timestamp)
{
  constexpr int width = 1440;
  constexpr int height = 1080;

  cv::Mat frame(height, width, CV_8UC3, cv::Scalar(10, 14, 10));
  const cv::Scalar grid_color(26, 34, 26);
  for (int x = 0; x < width; x += 80) {
    cv::line(frame, cv::Point(x, 0), cv::Point(x, height - 1), grid_color, 1, cv::LINE_AA);
  }
  for (int y = 0; y < height; y += 80) {
    cv::line(frame, cv::Point(0, y), cv::Point(width - 1, y), grid_color, 1, cv::LINE_AA);
  }

  const cv::Point center(width / 2, height / 2);
  const cv::Rect roi(center.x - 210, center.y - 210, 420, 420);
  cv::rectangle(frame, roi, cv::Scalar(30, 48, 30), cv::FILLED);
  cv::rectangle(frame, roi, cv::Scalar(120, 190, 120), 2, cv::LINE_AA);
  for (int offset = 30; offset < roi.width; offset += 30) {
    cv::line(
      frame,
      cv::Point(roi.x + offset, roi.y),
      cv::Point(roi.x + offset, roi.y + roi.height),
      cv::Scalar(75, 115, 75),
      1,
      cv::LINE_AA);
    cv::line(
      frame,
      cv::Point(roi.x, roi.y + offset),
      cv::Point(roi.x + roi.width, roi.y + offset),
      cv::Scalar(75, 115, 75),
      1,
      cv::LINE_AA);
  }
  cv::circle(frame, center, 120, cv::Scalar(180, 220, 180), 2, cv::LINE_AA);
  cv::circle(frame, center, 70, cv::Scalar(210, 240, 210), 2, cv::LINE_AA);
  cv::line(frame, cv::Point(center.x - 150, center.y), cv::Point(center.x + 150, center.y), cv::Scalar(240, 255, 240), 2, cv::LINE_AA);
  cv::line(frame, cv::Point(center.x, center.y - 150), cv::Point(center.x, center.y + 150), cv::Scalar(240, 255, 240), 2, cv::LINE_AA);
  cv::putText(frame, "ROI", cv::Point(roi.x + 130, roi.y + 205), cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(235, 255, 235), 3, cv::LINE_AA);
  cv::putText(frame, std::to_string(sequence % 1000), cv::Point(roi.x + 145, roi.y + 255), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(235, 255, 235), 2, cv::LINE_AA);

  const int projectile_phase = static_cast<int>((sequence * 18U) % 360U);
  const int projectile_x = roi.x + 30 + projectile_phase;
  const int projectile_y = center.y - 90 + static_cast<int>((sequence * 7U) % 180U);
  cv::circle(frame, cv::Point(projectile_x, projectile_y), 14, cv::Scalar(240, 245, 255), cv::FILLED, cv::LINE_AA);
  cv::circle(frame, cv::Point(projectile_x, projectile_y), 26, cv::Scalar(90, 150, 255), 2, cv::LINE_AA);

  const int target_x = 160 + static_cast<int>((sequence * 9U) % 640U);
  cv::rectangle(frame, cv::Rect(target_x, 760, 120, 150), cv::Scalar(60, 90, 210), cv::FILLED);
  cv::rectangle(frame, cv::Rect(target_x + 25, 700, 70, 70), cv::Scalar(120, 170, 255), cv::FILLED);

  FrameInfo result;
  result.width = static_cast<uint16_t>(width);
  result.height = static_cast<uint16_t>(height);
  result.sequence = sequence;
  result.timestamp = timestamp;
  result.bytes = static_cast<uint32_t>(frame.total() * frame.elemSize());
  result.rgb24.assign(frame.datastart, frame.dataend);
  return result;
}

}  // namespace bridge
