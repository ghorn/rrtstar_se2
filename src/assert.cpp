#include <cstdlib>
#include <mutex>

namespace rrts {

std::mutex g_exit_mutex;

[[noreturn]] void ExitThreadSafe(int exit_code) {
  const std::lock_guard<std::mutex> lock(g_exit_mutex);
  std::exit(exit_code);  // NOLINT
}

};  // namespace rrts
