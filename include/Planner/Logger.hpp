#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <chrono>
#include <iostream>

enum LLEVEL { LOG_ERROR = 0, LOG_WARNING = 1, LOG_INFO = 2, LOG_DEBUG = 3};
void rlog(std::string func, LLEVEL log_level, std::string text, bool disable = false);

#endif