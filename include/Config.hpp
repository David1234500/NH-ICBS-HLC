#ifndef CONFIG_HEADER_HPP
#define CONFIG_HEADER_HPP

#define PI 3.14159265f

 extern int map_size_x ;
 extern int map_size_y ;
 extern int map_size_angle ;
 extern int map_size_speed ;

 extern float timestep_ms ;

 extern int map_size_x_cm ;
 extern int map_size_y_cm ;

 extern float xpc;
 extern float ypc;
 extern float api;

 extern float safe_radius;
 extern float safe_level2_rad;
 extern float heuristic_factor_backwards;

 extern int zero_velocity_level;
 extern float m_speedsFactor[3];
 extern bool m_speedFactorIntermediate[3];

/* Constraextern ints what the allow angle range is for the edge */
 extern float state_change_fit_quality_angle;
 extern float state_change_fit_quality_position;
 extern float state_change_fit_allowed_speed_difference;

 extern float state_change_fit_threshold_angle_index_difference;
 extern float state_change_fit_threshold_angle_difference;

 extern int worker_counter;

 void recompute_inferred_values();

#endif //test@test.com::MyPassword
// /backend/node_modules/config/lib/config.js:182
//https://rwth.zoom.us/j/94334747430?pwd=S2F3czl1N1VuN28zUTZKUUxtTG8rZz09
