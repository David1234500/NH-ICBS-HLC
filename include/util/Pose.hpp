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




struct PBIConstraint{
    int32_t x = 0;
    int32_t y = 0;
    int32_t a = 0;
    int32_t s = 0;
    int32_t t = 0;
    int32_t id = 0;

    inline void operator=(PoseByIndex e) {
       x = e.x; 
       y = e.y; 
       a = e.a;
       s = e.s;
    }

    inline bool operator==(PoseByIndex e) {
        if(e.x==x && e.y == y && e.a == a && e.s== s){
            return true;
        }else{
            return false;
        }
    }

};

struct LLNode{
    PoseByIndex pose;
    uint32_t timestep = 0;
};

struct Pose2D{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f}; 
    
    /* Heading in radians */
    float h = 0.f; 

    /* Velocity in m/s */
    float vel = 0.f;
};

struct Pose2DWithError{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f};
    
    /* Heading in radians and index */
    float h = 0.f; 

    /* Velocity in m/s and node index */
    float vel = 0.f;

    PoseByIndex bi_pose;

    float a_error = 0.f;
    float p_error = 0.f;

    /* settings values for the associated fit */
    float s_a = 0.f;
    float s_v = 0.f;

    float s_a_2 = 0.f;
    float s_v_2 = 0.f;
};

}

}


namespace std {
    template<> struct hash<dynamics::data::PoseByIndex>
    {
        std::size_t operator()(const dynamics::data::PoseByIndex& p) const noexcept
        {
            return p.s + p.a * 10 + 100 * p.y + 10000 * p.x;
        }
    };
}
#endif //UTIL_POSE_HPP