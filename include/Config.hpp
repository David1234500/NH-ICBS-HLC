#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#include <DynamicsModel/SingleTrackModel.hpp>

constexpr int map_size_x = 60;
constexpr int map_size_y = 60;
constexpr int map_size_angle = 12;
constexpr int map_size_speed = 4;

constexpr float timestep_ms = 250.f;

constexpr int map_size_x_cm = 400;
constexpr int map_size_y_cm = 400;

constexpr float xpc = (float)map_size_x_cm / (float)map_size_x;
constexpr float ypc = (float)map_size_y_cm / (float)map_size_y;
constexpr float api = (2.f * PI) / (float)map_size_angle;

constexpr float m_speedsFactor[4] = {0.075f, 0.125f, 0.25f, 0.5f}; //-0.5f, -0.25f

constexpr uint32_t worker_counter = 28;

#endif