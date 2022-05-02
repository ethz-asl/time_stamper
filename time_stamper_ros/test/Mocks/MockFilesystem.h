#pragma once
class MockFilesystem : public IFileSystem {
 public:
  MOCK_METHOD2(write, bool(const std::string &path, const std::string &message));
  MOCK_METHOD3(read, bool(const std::string &path, void *buffer, size_t buffer_size));
  MOCK_METHOD1(directoryExists, bool(const char *path));
};