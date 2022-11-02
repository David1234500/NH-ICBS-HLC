#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#define PI 3.14159265f


constexpr double safe_radius = 30.f;
constexpr double heuristic_factor_backwards = 2.f;

constexpr int zero_velocity_level = 1;



/* Constraints what the allow angle range is for the edge */
constexpr double state_change_fit_quality_angle = 0.3f;
constexpr double state_change_fit_quality_position =  0.025f;


constexpr double state_change_fit_allowed_speed_difference = 0.15f;
;

constexpr int worker_counter = 26;

#endif //test@test.com::MyPassword
// /backend/node_modules/config/lib/config.js:182
//https://rwth.zoom.us/j/94334747430?pwd=S2F3czl1N1VuN28zUTZKUUxtTG8rZz09