#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#define PI 3.14159265f

constexpr int map_size_x = 60;
constexpr int map_size_y = 60;
constexpr int map_size_angle = 8;
constexpr int map_size_speed = 6;

constexpr float timestep_ms = 500.f;

constexpr int map_size_x_cm = 450;
constexpr int map_size_y_cm = 400;

constexpr float xpc = (float)map_size_x_cm / (float)map_size_x;
constexpr float ypc = (float)map_size_y_cm / (float)map_size_y;
constexpr float api = (2.f * PI) / (float)map_size_angle;

constexpr float safe_radius = 15.f;
constexpr float heuristic_factor_backwards = 3.f;

constexpr int zero_velocity_level = 1;
constexpr float m_speedsFactor[map_size_speed] = { -0.3f, -0.15f, 0.f, 0.15f, 0.3f, 0.6f}; 


/* Constraints what the allow angle range is for the edge */
constexpr float state_change_fit_quality_angle = api / 2;
constexpr float state_change_fit_quality_position =  xpc / 2;
constexpr float state_change_fit_allowed_speed_difference = 0.15f;

constexpr float state_change_fit_threshold_angle_index_difference = 3;
constexpr float state_change_fit_threshold_angle_difference = 0.5f * PI;

constexpr int worker_counter = 30;

#endif //test@test.com::MyPassword
// /backend/node_modules/config/lib/config.js:182
//https://rwth.zoom.us/j/94334747430?pwd=S2F3czl1N1VuN28zUTZKUUxtTG8rZz09
