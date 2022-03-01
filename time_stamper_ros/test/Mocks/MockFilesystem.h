#pragma once
class MockFilesystem : public IFileSystem {
 public:
  MOCK_METHOD2(Write, bool(const std::string &path, const std::string &message));
  MOCK_METHOD3(Read, bool(const std::string &path, void *buffer, size_t buffer_size));
  MOCK_METHOD1(DirectoryExists, bool(const char *path));
};