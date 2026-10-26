// 有界 JSONL 轮转：当前日志加固定数量备份，独占锁保护整个文件族。
#include "demo_aim/detail/app/json_log.hpp"
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace aim::app {
namespace {
std::runtime_error fileError(const std::string &action,
                            const std::string &path, int error) {
  return std::runtime_error(action + ": " + path + ": " +
                            std::strerror(error));
}

// 不跟随符号链接，也不把目录、设备或管道当成可淘汰的日志。
bool regularOrMissing(const std::string &path) {
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  if (error && error != std::errc::no_such_file_or_directory)
    throw std::runtime_error("Cannot inspect output: " + path + ": " +
                             error.message());
  if (status.type() == std::filesystem::file_type::not_found)
    return false;
  if (!std::filesystem::is_regular_file(status))
    throw std::runtime_error("Output must be a regular file: " + path);
  return true;
}
} // namespace

JsonLog::JsonLog(const std::string &requested, JsonLogLimits limits)
    : path_(requested), limits_(limits) {
  const std::filesystem::path original(path_);
  if (original.filename().empty() || original.filename() == "." ||
      original.filename() == "..")
    throw std::invalid_argument("Set a log file path");
  if (original.extension() == ".lock")
    throw std::invalid_argument("Log paths ending in .lock are reserved");
  if (limits_.max_file_bytes == 0 || limits_.max_files == 0 ||
      limits_.max_files > 1000)
    throw std::invalid_argument("Invalid log limits");
  const auto parent = original.parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);
  // 同时占用备份名称，避免另一个写者把 FILE.1 当作自己的当前文件。
  lock_fds_.reserve(limits_.max_files);
  try {
    for (unsigned index = 0; index < limits_.max_files; ++index) {
      const auto log_path = index ? backupPath(index) : path_;
      const auto lock_path = log_path + ".lock";
      const int fd = ::open(lock_path.c_str(),
                            O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW |
                                O_NONBLOCK,
                            0644);
      if (fd < 0)
        throw fileError("Cannot open log lock", lock_path, errno);
      lock_fds_.push_back(fd);
      struct stat status {};
      if (::fstat(fd, &status) != 0)
        throw fileError("Cannot inspect log lock", lock_path, errno);
      if (!S_ISREG(status.st_mode))
        throw std::runtime_error("Log lock must be a regular file: " + lock_path);
      if (::flock(fd, LOCK_EX | LOCK_NB) != 0)
        throw fileError("Log output already in use or cannot be locked",
                        log_path, errno);
    }
    rotate();
  } catch (...) {
    for (const int fd : lock_fds_)
      ::close(fd);
    throw;
  }
}

JsonLog::~JsonLog() {
  if (file_)
    std::fclose(file_);
  // 锁文件保留：删除它会让新进程锁到另一个 inode，绕过现存锁。
  for (const int fd : lock_fds_)
    ::close(fd);
}

std::string JsonLog::backupPath(unsigned index) const {
  return path_ + "." + std::to_string(index);
}

void JsonLog::rotate() {
  // 先检查所有将被操作的路径，防止发现异常备份前已淘汰正常记录。
  const bool current_exists = regularOrMissing(path_);
  for (unsigned index = 1; index < limits_.max_files; ++index)
    regularOrMissing(backupPath(index));
  if (file_) {
    auto *closing = file_;
    file_ = nullptr;
    if (std::fclose(closing) != 0)
      throw fileError("Cannot close output", path_, errno);
  }
  if (current_exists) {
    if (limits_.max_files == 1) {
      std::filesystem::remove(path_);
    } else {
      std::filesystem::remove(backupPath(limits_.max_files - 1));
      for (unsigned index = limits_.max_files - 1; index > 1; --index) {
        const auto previous = backupPath(index - 1);
        if (regularOrMissing(previous))
          std::filesystem::rename(previous, backupPath(index));
      }
      std::filesystem::rename(path_, backupPath(1));
    }
  }
  // O_EXCL 避免截断意外出现的文件；O_NOFOLLOW 不写入链接指向的内容。
  const int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL |
                                         O_CLOEXEC | O_NOFOLLOW,
                        0644);
  if (fd < 0)
    throw fileError("Cannot create output", path_, errno);
  file_ = ::fdopen(fd, "w");
  if (!file_) {
    const int error = errno;
    ::close(fd);
    throw fileError("Cannot open output stream", path_, error);
  }
  bytes_written_ = 0;
}

// fflush 不等同于 fsync 的断电持久性保证。
void JsonLog::write(const std::string &text) {
  if (!file_)
    throw std::runtime_error("Output is closed: " + path_);
  // 计入换行，先拒绝超大单条，避免循环轮转或留下半行 JSON。
  if (text.size() >= limits_.max_file_bytes)
    throw std::runtime_error("Log record exceeds file size limit: " + path_);
  const auto record_bytes = text.size() + 1;
  if (record_bytes > limits_.max_file_bytes - bytes_written_)
    rotate();
  if (std::fwrite(text.data(), 1, text.size(), file_) != text.size() ||
      std::fputc('\n', file_) == EOF || std::fflush(file_) != 0) {
    const int error = errno ? errno : EIO;
    auto *failed = file_;
    file_ = nullptr;
    std::fclose(failed);
    throw fileError("Output write failed", path_, error);
  }
  bytes_written_ += record_bytes;
}
} // namespace aim::app
