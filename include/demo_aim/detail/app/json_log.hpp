// JSONL 轮转输出：限制单份大小和日志总数，同一路径只允许一个写入者。
#pragma once
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>
namespace aim::app {
struct JsonLogLimits {
  std::size_t max_file_bytes{10 * 1024 * 1024};
  unsigned max_files{5}; // 包括当前文件；其余为 .1、.2 等历史文件。
};
// RAII 管理 FILE 句柄；禁止复制，防止重复关闭同一个文件。
class JsonLog {
public:
  explicit JsonLog(const std::string &requested, JsonLogLimits limits = {});
  ~JsonLog();
  JsonLog(const JsonLog &) = delete;
  JsonLog &operator=(const JsonLog &) = delete;
  // 当前日志始终使用请求路径；.1 是最近一份历史文件。
  const std::string &path() const { return path_; }
  // 按完整行轮转、加换行并刷新；超大单行或写盘失败抛异常。
  void write(const std::string &text);

private:
  void rotate();
  std::string backupPath(unsigned index) const;
  std::FILE *file_{};
  std::string path_;
  JsonLogLimits limits_;
  std::size_t bytes_written_{};
  std::vector<int> lock_fds_;
};
} // namespace aim::app
