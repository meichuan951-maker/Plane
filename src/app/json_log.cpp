// 日志实现：使用独占创建避免覆盖已有记录，也避免并发进程写入同一路径。
#include "json_log.hpp"
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
namespace aim::app {
  // 先尝试原名，冲突时依次尝试 stem_1.ext 等；编号最多试到 10000。
  JsonLog::JsonLog(const std::string &requested) {
    const std::filesystem::path original(requested);
    const auto parent = original.parent_path();
    if (!parent.empty())
      std::filesystem::create_directories(parent);
    // Exclusive creation also prevents concurrent runs from overwriting logs.
    for (unsigned suffix = 0; suffix <= 10000; ++suffix) {
      path_ = suffix == 0 ? original.string()
                          : (parent / (original.stem().string() + "_" +
                                      std::to_string(suffix) +
                                      original.extension().string()))
                                .string();
  // wx 保证创建与占用检查是同一次操作，不使用容易产生竞争的 exists 后再写入。
      file_ = std::fopen(path_.c_str(), "wx");
      if (file_) {
        if (suffix != 0)
          std::cerr << "Output already exists; writing to " << path_ << '\n';
        return;
      }
  // 仅 EEXIST 触发改名；权限不足、磁盘等其他错误直接报告。
      const int error = errno;
      if (error != EEXIST)
        throw std::runtime_error("Cannot create output: " + path_ + ": " +
                                std::strerror(error));
    }
    throw std::runtime_error("Too many existing output files for: " + requested);
  }
  // 退出作用域时关闭文件；逐条写入失败由 write 检查。
  JsonLog::~JsonLog() {
    if (file_)
      std::fclose(file_);
  }
  // 每条 JSON 都以换行结束并 fflush；这不等同于 fsync 的断电持久性保证。
  void JsonLog::write(const std::string &text) {
    if (std::fwrite(text.data(), 1, text.size(), file_) != text.size() ||
        std::fputc('\n', file_) == EOF || std::fflush(file_) != 0)
      throw std::runtime_error("Output write failed");
  }
} // namespace aim::app
