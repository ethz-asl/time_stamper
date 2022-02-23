#pragma once

class TimestampManager {
 public:
  /**
   * Default constructor.
   */
  TimestampManager();

  /**
   * Polls timestamps. If new timestamp is available, it gets moved to @var last_timestamp_
   * @return true if new timestamp is available otherwise false
   */
  bool Poll();

  /**
   *
   * @return value of last timestamp
   */
  double GetLastTimestamp() const;

  /**
   * Default destructor.
   */
  ~TimestampManager();

 private:
  /**
   * Stores last timestamp from poll()
   */
  double last_timestamp_{};

  /**
   * Default buffer size
   */
  static constexpr int BUFFER_SIZE = 4096;
};

