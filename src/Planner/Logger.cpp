#include <Planner/Logger.hpp>
#include <mutex>
#include <thread>
#include <set>

std::mutex log_mutex;

void rlog(std::string func, LLEVEL log_level, std::string text, bool disable) {
  if(disable){
    return;
  }

  log_mutex.lock();

  std::chrono::milliseconds msc =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  uint32_t ms = msc.count();
  switch (log_level) {
    case LOG_ERROR:
      std::cout << "[ERROR][" << std::this_thread::get_id() << "][" << ms << "-"
                << func << "] " << text << std::endl;
      break;
    case LOG_WARNING:
      std::cout << "[WARN][" << std::this_thread::get_id() << "][" << ms << "-"
                << func << "] " << text << std::endl;
      break;
    case LOG_INFO:
      std::cout << "[INFO][" << std::this_thread::get_id() << "][" << ms << "-"
                << func << "] " << text << std::endl;
      break;
  }
  log_mutex.unlock();
}