#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#define PI 3.14159265f

constexpr int map_size_x = 90;
constexpr int map_size_y = 90;
constexpr int map_size_angle = 8;
constexpr int map_size_speed = 1;



constexpr int map_size_x_cm = 450;
constexpr int map_size_y_cm = 400;


constexpr double safe_radius = 30.f;
constexpr double heuristic_factor_backwards = 2.f;

constexpr int zero_velocity_level = 1;



/* Constraints what the allow angle range is for the edge */
constexpr double state_change_fit_quality_angle = 0.3f;
constexpr double state_change_fit_quality_position =  0.3f;
constexpr double state_change_fit_allowed_speed_difference = 0.15f;
constexpr double state_change_fit_threshold_angle_index_difference = 3;
constexpr double state_change_fit_threshold_angle_difference = 0.5f * PI;

constexpr int worker_counter = 26;

#endif //test@test.com::MyPassword
// /backend/node_modules/config/lib/config.js:182
//https://rwth.zoom.us/j/94334747430?pwd=S2F3czl1N1VuN28zUTZKUUxtTG8rZz09