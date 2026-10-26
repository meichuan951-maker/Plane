#include "demo_aim/detail/app/json_log.hpp"
#include "support/assertions.hpp"
#include "support/runner.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {
namespace fs = std::filesystem;
using aim::app::JsonLog;
using aim::app::JsonLogLimits;
using Json = nlohmann::json;
using test_support::check;
using test_support::expectThrows;

struct TemporaryDirectory {
  fs::path path;
  TemporaryDirectory() {
    auto pattern = (fs::temp_directory_path() / "json-log-XXXXXX").string();
    const char *created = mkdtemp(pattern.data());
    if (!created)
      throw std::runtime_error("Cannot create JSON log fixture directory");
    path = created;
  }
  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(path, error);
  }
};

fs::path backup(const fs::path &path, unsigned index) {
  return path.string() + "." + std::to_string(index);
}

std::string row(int sequence) { return Json{{"sequence", sequence}}.dump(); }

std::string read(const fs::path &path) {
  std::ifstream stream(path, std::ios::binary);
  check(stream.is_open(), "Missing file: " + path.string());
  return {std::istreambuf_iterator<char>(stream),
          std::istreambuf_iterator<char>()};
}

void seed(const fs::path &path, const std::string &text) {
  std::ofstream stream(path, std::ios::binary);
  check(stream.is_open(), "Cannot create fixture: " + path.string());
  stream << text;
  check(stream.good(), "Cannot write fixture: " + path.string());
}

std::vector<int> sequences(const fs::path &path) {
  const auto text = read(path);
  check(!text.empty() && text.back() == '\n',
        "JSON log ended with an incomplete line: " + path.string());
  std::vector<int> result;
  std::size_t begin = 0;
  while (begin < text.size()) {
    const auto end = text.find('\n', begin);
    check(end != std::string::npos && end != begin,
          "JSON log contained a partial or empty line");
    const auto value = Json::parse(text.substr(begin, end - begin));
    check(value.is_object() && value.at("sequence").is_number_integer(),
          "JSON log changed a record");
    result.push_back(value.at("sequence").get<int>());
    begin = end + 1;
  }
  return result;
}

void retainedRuns() {
  for (unsigned max_files : {1u, 3u}) {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "run.jsonl";
    for (int sequence = 0; sequence < 7; ++sequence) {
      {
        JsonLog log(path.string(), JsonLogLimits{1024, max_files});
        check(log.path() == path.string(), "Current log path changed");
        log.write(row(sequence));
      }
      check(sequences(path) == std::vector<int>{sequence},
            "A new run did not start a fresh current log");
      for (unsigned index = 1; index < max_files; ++index) {
        if (index <= static_cast<unsigned>(sequence))
          check(sequences(backup(path, index)) ==
                    std::vector<int>{sequence - static_cast<int>(index)},
                "Retained runs are out of order");
        else
          check(!fs::exists(backup(path, index)),
                "Rotation created a history without a prior run");
      }
      check(!fs::exists(backup(path, max_files)),
            "Log history exceeded the configured count");
      check(fs::is_regular_file(path.string() + ".lock"),
            "Family lock disappeared after the writer closed");
    }
  }
}

void completeLinesAtSizeBoundary() {
  TemporaryDirectory temporary;
  const auto path = temporary.path / "run.jsonl";
  const auto limit = row(0).size() + row(1).size() + 2;
  JsonLog log(path.string(), JsonLogLimits{limit, 3});
  log.write(row(0));
  log.write(row(1));
  check(fs::file_size(path) == limit && !fs::exists(backup(path, 1)),
        "A record that exactly fits caused premature rotation");
  check(sequences(path) == std::vector<int>({0, 1}),
        "Records were split at the exact size boundary");
  log.write(row(2));
  check(sequences(backup(path, 1)) == std::vector<int>({0, 1}) &&
            sequences(path) == std::vector<int>{2},
        "Rotation did not preserve complete records");
  for (int sequence = 3; sequence < 7; ++sequence)
    log.write(row(sequence));
  check(sequences(path) == std::vector<int>{6} &&
            sequences(backup(path, 1)) == std::vector<int>({4, 5}) &&
            sequences(backup(path, 2)) == std::vector<int>({2, 3}) &&
            !fs::exists(backup(path, 3)),
        "Size rotation did not keep the newest bounded record groups");
  for (unsigned index = 0; index < 3; ++index)
    check(fs::file_size(index ? backup(path, index) : path) <= limit,
          "Retained log exceeded the configured byte limit");
  check(log.path() == path.string(), "Size rotation changed the current path");
}

void oversizedRecordLeavesFilesUnchanged() {
  TemporaryDirectory temporary;
  const auto path = temporary.path / "run.jsonl";
  const auto limit = row(0).size() + 1;
  JsonLog log(path.string(), JsonLogLimits{limit, 3});
  log.write(row(0));
  log.write(row(1));
  const auto current = read(path), history = read(backup(path, 1));
  const auto too_large = Json{{"message", std::string(limit, 'x')}}.dump();
  expectThrows([&] { log.write(too_large); });
  check(read(path) == current && read(backup(path, 1)) == history &&
            !fs::exists(backup(path, 2)),
        "An oversized record modified or rotated existing logs");
  log.write(row(2));
  check(sequences(path) == std::vector<int>{2} &&
            sequences(backup(path, 1)) == std::vector<int>{1} &&
            sequences(backup(path, 2)) == std::vector<int>{0},
        "Writer could not continue after rejecting an oversized record");
}

void onlyExactFamilyIsRotated() {
  TemporaryDirectory temporary;
  const auto path = temporary.path / "run.jsonl";
  const std::vector<std::string> unrelated{
      "run_1.jsonl", "run_999.jsonl", "other.jsonl", "run.jsonl.01",
      "run.jsonl.backup", "run.jsonl.1.extra", "run.jsonl-old"};
  for (const auto &name : unrelated)
    seed(temporary.path / name, "unrelated:" + name);
  const auto directory = temporary.path / "run.jsonl.extra";
  fs::create_directory(directory);
  seed(directory / "sentinel", "keep directory contents");
  const auto target = temporary.path / "neighbor-target";
  seed(target, "keep symlink target");
  const auto neighbor_link = temporary.path / "run.jsonl.similar";
  fs::create_symlink(target, neighbor_link);

  for (int sequence = 0; sequence < 6; ++sequence) {
    JsonLog log(path.string(), JsonLogLimits{1024, 3});
    log.write(row(sequence));
  }
  for (const auto &name : unrelated)
    check(read(temporary.path / name) == "unrelated:" + name,
          "Rotation modified an unrelated file: " + name);
  check(read(directory / "sentinel") == "keep directory contents" &&
            fs::is_symlink(neighbor_link) && read(target) == "keep symlink target",
        "Rotation traversed or removed a neighboring directory or symlink");
}

void secondProcessCannotReplaceActiveLog() {
  TemporaryDirectory temporary;
  const auto path = temporary.path / "run.jsonl";
  {
    JsonLog log(path.string(), JsonLogLimits{1024, 3});
    log.write(row(0));
    const pid_t child = fork();
    check(child >= 0, "Cannot fork competing log writer");
    if (child == 0) {
      try {
        JsonLog competing(path.string(), JsonLogLimits{1024, 3});
        competing.write(row(99));
        _exit(2);
      } catch (const std::exception &) {
        _exit(0);
      }
    }
    int status = 0;
    pid_t done = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      done = waitpid(child, &status, WNOHANG);
      if (done != 0)
        break;
      usleep(5000);
    }
    if (done == 0) {
      kill(child, SIGKILL);
      waitpid(child, &status, 0);
      throw std::runtime_error(
          "Competing writer blocked instead of rejecting the active lock");
    }
    check(done == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "Competing process acquired an active log family");
    check(sequences(path) == std::vector<int>{0} &&
              !fs::exists(backup(path, 1)),
          "Competing writer modified the active log");
    log.write(row(1));
  }
  check(fs::is_regular_file(path.string() + ".lock"),
        "Writer removed the persistent lock file");
  {
    JsonLog next(path.string(), JsonLogLimits{1024, 3});
    next.write(row(2));
  }
  check(sequences(path) == std::vector<int>{2} &&
            sequences(backup(path, 1)) == std::vector<int>({0, 1}),
        "Family lock was not released when its writer closed");
}

void overlappingFamiliesCannotReplaceActiveLogs() {
  const JsonLogLimits limits{1024, 3};
  {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "run.jsonl";
    seed(path, row(-1) + "\n");
    seed(backup(path, 1), row(-2) + "\n");
    seed(backup(path, 2), row(-3) + "\n");
    {
      JsonLog base(path.string(), limits);
      base.write(row(0));
      const auto current = read(path);
      const auto first = read(backup(path, 1));
      const auto second = read(backup(path, 2));
      expectThrows([&] { JsonLog nested(backup(path, 2).string(), limits); });
      check(read(path) == current && read(backup(path, 1)) == first &&
                read(backup(path, 2)) == second &&
                !fs::exists(backup(path, 2).string() + ".1"),
            "A backup-named writer changed an active log family");
    }
    {
      JsonLog next(path.string(), limits);
      next.write(row(1));
    }
    check(sequences(path) == std::vector<int>{1} &&
              sequences(backup(path, 1)) == std::vector<int>{0},
          "Base family locks remained held after the first writer closed");
  }
  {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "run.jsonl";
    seed(path, row(-1) + "\n");
    {
      JsonLog nested(backup(path, 2).string(), limits);
      nested.write(row(99));
      const auto current = read(path);
      const auto nested_current = read(backup(path, 2));
      expectThrows([&] { JsonLog base(path.string(), limits); });
      check(read(path) == current && read(backup(path, 2)) == nested_current &&
                !fs::exists(backup(path, 1)),
            "Base rotation changed a live backup-named writer or prior logs");
      nested.write(row(100));
    }
    {
      JsonLog next(path.string(), limits);
      next.write(row(1));
    }
    check(sequences(path) == std::vector<int>{1} &&
              sequences(backup(path, 1)) == std::vector<int>{-1},
          "Failed overlapping construction leaked partially acquired locks");
  }
}

void unsafeFamilyEntriesAreRejected(bool symlink) {
  for (const char *suffix_text : {"", ".1", ".2", ".lock", ".1.lock",
                                  ".2.lock"}) {
    const std::string suffix(suffix_text);
    TemporaryDirectory temporary;
    const auto path = temporary.path / "run.jsonl";
    const fs::path unsafe = path.string() + suffix;
    const auto victim = temporary.path / "victim";
    seed(victim, "never alter this file");
    if (suffix != "")
      seed(path, row(0) + "\n");
    if (symlink)
      fs::create_symlink(victim, unsafe);
    else {
      fs::create_directory(unsafe);
      seed(unsafe / "sentinel", "keep directory contents");
    }

    expectThrows([&] { JsonLog log(path.string(), JsonLogLimits{1024, 3}); });
    check(read(victim) == "never alter this file",
          "Unsafe family entry changed its symlink target");
    check(symlink ? fs::is_symlink(unsafe) : fs::is_directory(unsafe),
          "Unsafe family entry was renamed or removed");
    if (!symlink)
      check(read(unsafe / "sentinel") == "keep directory contents",
            "Rotation removed contents of a non-file family entry");
    if (suffix != "")
      check(read(path) == row(0) + "\n",
            "Rejected log construction modified the prior current log");
  }
}
} // namespace

int main() {
  test_support::Runner test;
  test("runs_keep_only_the_newest_configured_files", retainedRuns);
  test("size_rotation_keeps_complete_json_lines", completeLinesAtSizeBoundary);
  test("oversized_record_does_not_modify_logs",
       oversizedRecordLeavesFilesUnchanged);
  test("rotation_preserves_other_log_families", onlyExactFamilyIsRotated);
  test("lock_metadata_cannot_be_used_as_log_output", [] {
    TemporaryDirectory temporary;
    const auto path = temporary.path / "run.jsonl";
    JsonLog log(path.string(), JsonLogLimits{1024, 3});
    log.write(row(0));
    expectThrows([&] { JsonLog other(path.string() + ".lock"); });
    expectThrows([&] { JsonLog other(path.string() + ".1.lock"); });
    expectThrows([&] { JsonLog competing(path.string()); });
    check(sequences(path) == std::vector<int>{0} &&
              !fs::exists(backup(path, 1)),
          "Log output replaced lock metadata or changed the active file");
  });
  test("active_family_is_exclusive_across_processes",
       secondProcessCannotReplaceActiveLog);
  test("overlapping_families_preserve_active_logs_in_both_directions",
       overlappingFamiliesCannotReplaceActiveLogs);
  test("symlink_family_entries_are_rejected",
       [] { unsafeFamilyEntriesAreRejected(true); });
  test("non_file_family_entries_are_rejected",
       [] { unsafeFamilyEntriesAreRejected(false); });
  return test.finish();
}
