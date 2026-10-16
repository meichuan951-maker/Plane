// JSONL 文件输出接口：每次运行创建独立日志，重名自动编号。
#pragma once
#include <cstdio>
#include <string>
namespace aim::app {
// RAII 管理 FILE 句柄；禁止复制，防止重复关闭同一个文件。
class JsonLog {
public:
  explicit JsonLog(const std::string &requested);
  ~JsonLog();
  JsonLog(const JsonLog &) = delete;
  JsonLog &operator=(const JsonLog &) = delete;
  // 返回实际创建的路径，可能与请求路径不同。
  const std::string &path() const { return path_; }
  // 写入一条文本并加换行、刷新用户态缓冲；写盘失败抛异常。
  void write(const std::string &text);

private:
  std::FILE *file_{};
  std::string path_;
};
} // namespace aim::app
