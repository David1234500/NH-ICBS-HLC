#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#include <DynamicsModel/SingleTrackModel.hpp>

constexpr int map_size_x = 50;
constexpr int map_size_y = 50;
constexpr int map_size_angle = 32;
constexpr int map_size_speed = 5;

constexpr float timestep_ms = 250.f;

constexpr int map_size_x_cm = 400;
constexpr int map_size_y_cm = 400;

constexpr float xpc = (float)map_size_x_cm / (float)map_size_x;
constexpr float ypc = (float)map_size_y_cm / (float)map_size_y;
constexpr float api = (2.f * PI) / (float)map_size_angle;

constexpr float m_speedsFactor[5] = {-0.5f, -0.25f, 0.25f, 0.5f, 1.0f};

#endif