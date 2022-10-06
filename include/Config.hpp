#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#define PI 3.14159265f

constexpr int map_size_x = 200;
constexpr int map_size_y = 200;
constexpr int map_size_angle = 8;
constexpr int map_size_speed = 4;

constexpr float timestep_ms = 300.f;

constexpr int map_size_x_cm = 450;
constexpr int map_size_y_cm = 400;

constexpr float xpc = (float)map_size_x_cm / (float)map_size_x;
constexpr float ypc = (float)map_size_y_cm / (float)map_size_y;
constexpr float api = (2.f * PI) / (float)map_size_angle;

constexpr float safe_radius = 30.f;
constexpr float heuristic_factor_backwards = 2.f;

constexpr float m_speedsFactor[map_size_speed] = { -0.3, 0.f, 0.3f, 0.6f}; 

constexpr float state_change_fit_allowed_speed_difference = 0.125f;

/* Constraints what the allow angle range is for the edge */
constexpr float state_change_fit_quality_angle = api / 6;
constexpr float state_change_fit_quality_position =  xpc / 10;

constexpr int worker_counter = 28;

#endif