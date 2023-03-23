#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <chrono>
#include <iostream>

enum LLEVEL { LOG_ERROR, LOG_WARNING, LOG_INFO };
void rlog(std::string func, LLEVEL log_level, std::string text, bool disable = false);

#endif