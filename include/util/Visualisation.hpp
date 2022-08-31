#ifndef VIS_HPP
#define VIS_HPP

#include <Planner/SimplePlanner.hpp>
#include <util/Pose.hpp>
#include <cairo/cairo.h>

class Visualization{
public:
static void draw_pose(cairo_t *cr, cairo_surface_t *surface,  dynamics::Pose2D pose){
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_rectangle (cr, pose.pos[0] - 5, pose.pos[1] - 5, 10, 10);
    cairo_fill (cr);

    cairo_move_to (cr, pose.pos[0] - 5, pose.pos[1] - 5);
    dynamics::Vector2Df vec{10.0f,0.0f};
    Eigen::Rotation2Df r(pose.h);
    auto vec_rot = r*vec;
    cairo_line_to (cr, pose.pos[0] - 5 + vec_rot[0], pose.pos[1] - 5 + vec_rot[1]);
    cairo_stroke (cr);
};



};

#endif