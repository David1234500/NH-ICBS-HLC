#ifndef UTIL_POSE_HPP
#define UTIL_POSE_HPP
 
#include <Eigen/Dense>


namespace dynamics{

namespace data{

typedef Eigen::Matrix<float, 2, 1> Position2Df;
typedef Eigen::Matrix<float, 2, 1> Vector2Df;



struct PoseByIndex{
    int32_t x = 0;
    int32_t y = 0;
    int32_t a = 0;
    int32_t s = 0;

    inline PoseByIndex operator+(PoseByIndex e) {
        PoseByIndex n = {e.x + x, e.y + y, a,s};
        return n;
    }

    inline PoseByIndex operator+(int32_t offset) {
        PoseByIndex n = {x + offset, y + offset, a,s};
        return n;
    }

    inline PoseByIndex operator-(PoseByIndex e) {
        PoseByIndex n = {e.x - x, e.y - y,a,s};
        return n;
    }

     inline PoseByIndex operator-(int32_t offset) {
        PoseByIndex n = {x - offset, y - offset, a,s};
        return n;
    }

    bool operator==( const PoseByIndex& e) const {
        if(e.x==x && e.y == y && e.a == a && e.s== s){
            return true;
        }else{
            return false;
        }
    }

     bool operator < (const dynamics::data::PoseByIndex r) const {
        if(x < r.x){
            return true;
        }else if(x > r.x){
            return false;
        }else{
            if(y < r.y){
                return true;
            }else if(y > r.y){
                return false;
            }else{
                if(a < r.a){
                    return true;
                }else if(a > r.a){
                    return false;
                }else{
                    if(s < r.s){
                        return true;
                    }else{
                        return false;
                    }
                }
            }
        }
    }
};






struct LLNode{
    PoseByIndex pose;
    double fScore = 10000000.f;
    uint32_t timestep = 0;

    bool operator < (const dynamics::data::LLNode r) const {
        if(fScore < r.fScore){
            return false;
        }else{
            return true;
        }
    }
};

struct Pose2D{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f}; 
    
    /* Heading in radians */
    double h = 0.f; 

    /* Velocity in m/s */
    double vel = 0.f;
};

struct Pose2WithTime{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f}; 
    
    /* Heading in radians */
    double h = 0.f; 

    /* Velocity in m/s */
    double vel = 0.f;

    double time_ms = 0.f;

    inline void operator=(Pose2D e){
        pos = e.pos;
        h = e.h;
        vel = e.vel;
    }
};


struct Constraint{
    uint32_t vehicle_a;
    uint32_t vehicle_b;
    double time_ms;
    dynamics::data::Position2Df pos;  
};

struct Pose2DWithMotionData{
    Position2Df pos = {0.f,0.f};
    double h = 0.f; 
    double vel = 0.f;

    double start_vel = 0.f;
    double target_vel = 0.f;
    int32_t target_vel_index = 0;

    double s_a = 0.f;
    double s_a_2 = 0.f;
    uint32_t ts_ms = 0;

    inline void operator=(const Pose2D& e){
        pos = e.pos;
        h = e.h;
        vel = e.vel;
    }
};

}

}


namespace std {
    template<> struct hash<dynamics::data::PoseByIndex>
    {
        std::size_t operator()(const dynamics::data::PoseByIndex& p) const noexcept
        {
            
            // 4,294,967,295
            // 100000 * 100 < 4294967295
            // TODO: THROW SOME ASSERTS IN HERE FOR GOOD MEASURE, allowable interval should now be 1000x1000
            return p.s + p.a * 10 + 1000 * p.y + 1000000 * p.x;
        }
    };
}

namespace std {
    template<> struct hash<dynamics::data::PBIConstraint>
    {
        std::size_t operator()(const dynamics::data::PBIConstraint& p) const noexcept
        {
            // 4294967295
            // 10000000 * 100
            // = 10000000
            return p.s + p.a * 10 + 1000 * p.y + 100000 * p.x + 10000000 * p.t;
        }
    };
}
#endif //UTIL_POSE_HPP