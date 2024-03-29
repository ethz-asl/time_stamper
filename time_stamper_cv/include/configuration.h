#pragma once

/**
 * Describes a row of LEDs in a 2D coordinate system.
 *
 * @param first_led_pos
 * Position of the first LED in a 2D coordinate system.
 *
 * @param next_led
 * Vector from previous to next LED, thus every LED in a row has the same vector to the next LED.
 *
 * @param amount
 * Amount of LEDs in a row.
 *
 * @param multiplier
 * Can be used to scale coordinates.
 *
 */

struct LedRowConfig {
  cv::Point2f first_led_pos;
  cv::Vec2i next_led;
  int amount;
  int multiplier = 1;
};

/**
 * Configuration for image processing.
 *
 * @param tolerance
 * Creates a threshold for detecting angles.
 * (e.g. if set to 10, valid values for a 90 degree angle are 80-100)
 *
 * @param params
 * Parameters for the OpenCV SimpleBlobDetector. See OpenCV docs for more details.
 *
 * @param led_row_config
 * LedRowConfig with reference key.
 * See @struct LedRowConfig above for more details.
 */
typedef std::map<std::string, LedRowConfig> LedRowConfigRepository;

struct ImageProcessorConfig {
  inline static constexpr const char *TOP_ROW{"TopRow"};
  inline static constexpr const char *BOTTOM_ROW{"BottomRow"};

  int tolerance{10};
  cv::SimpleBlobDetector::Params params;
  LedRowConfigRepository led_row_config {
      {BOTTOM_ROW, {{0, 0}, {6, 0}, 16}},
      {TOP_ROW, {{0, 17}, {6, 0}, 10}}
  };

  /**
   * Get row names
   * @return string vector
   */
 public:
  [[nodiscard]] std::vector<std::string> rows() const {
    std::vector<std::string> res{};
    for (const auto& [item, _] : led_row_config) {
      res.push_back(item);
    }
    return res;
  }
};