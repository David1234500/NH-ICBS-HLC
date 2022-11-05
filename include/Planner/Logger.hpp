#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <chrono>
#include <iostream>

enum LLEVEL { LOG_ERROR, LOG_WARNING, LOG_INFO };
enum EXECPATH { NONE, ASTAR, GETPATH, CBS, GETSPLINE, FCONFLICT, ACONFLICT };

void rlog(std::string func, LLEVEL log_level, std::string text, EXECPATH path = NONE);

#endif