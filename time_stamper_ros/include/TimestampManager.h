#pragma once

class TimestampManager {
 public:
  TimestampManager();
  bool Poll();
  double GetLastTimestamp() const;
  ~TimestampManager();

 private:
  double last_timestamp_{};
  static const int BUFFER_SIZE = 4096;
};

