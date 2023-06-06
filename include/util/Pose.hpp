#ifndef UTIL_POSE_HPP
#define UTIL_POSE_HPP
 
#include <map>

#include <Eigen/Dense>

typedef std::size_t JointHash;
typedef int32_t VehicleID;

namespace dynamics{

namespace data{

typedef Eigen::Matrix<float, 2, 1> Position2Df;
typedef Eigen::Matrix<float, 2, 1> Vector2Df;



struct PoseByIndex{
    int32_t x = 0;
    int32_t y = 0;
    int32_t a = 0;
    int32_t s = 0;
    int32_t t = 0;

    inline PoseByIndex operator+(PoseByIndex e) {
        PoseByIndex n = {e.x + x, e.y + y, a, s, t};
        return n;
    }

    inline PoseByIndex operator+(int32_t offset) {
        PoseByIndex n = {x + offset, y + offset, a,s, t};
        return n;
    }

    inline PoseByIndex operator-(PoseByIndex e) {
        PoseByIndex n = {e.x - x, e.y - y,a,s,t };
        return n;
    }

     inline PoseByIndex operator-(int32_t offset) {
        PoseByIndex n = {x - offset, y - offset, a,s, t};
        return n;
    }

    bool operator==( const PoseByIndex& e) const {
        if(e.x==x && e.y == y && e.a == a && e.s== s && e.t==t){
            return true;
        }else{
            return false;
        }
    }

    bool operator&=( const PoseByIndex& e) const {
        if(e.x==x && e.y == y && e.a == a && e.s== s ){
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
                    }else if(s < r.s){
                        return false;
                    }else{
                        if(t < r.t){
                            return true;
                        }else{
                            return false;
                        }
                    }
                }
            }
        }
    }
    
};


struct MPoseByIndex{
    std::map<int32_t, dynamics::data::PoseByIndex> vehicle_poses;
    std::vector<int32_t> vehicle_indices;

    bool operator&=( MPoseByIndex e) {
        if(e.vehicle_poses.size() != vehicle_poses.size()){
            return false;
        }
        for(auto index: vehicle_indices){
            if(!(e.vehicle_poses[index] &= vehicle_poses[index])){
                return false;
            }
        }
        return true;
    }

     bool operator==( MPoseByIndex e) {
        if(e.vehicle_poses.size() != vehicle_poses.size()){
            return false;
        }
        for(auto index: vehicle_indices){
            if(!(e.vehicle_poses[index] == vehicle_poses[index])){
                return false;
            }
        }
        return true;
    }
     bool operator==(const MPoseByIndex e) const {
        if(e.vehicle_poses.size() != vehicle_poses.size()){
            return false;
        }
        for(auto index: vehicle_indices){
            if(!(e.vehicle_poses.at(index) == vehicle_poses.at(index))){
                return false;
            }
        }
        return true;
    }
};

struct PBIConstraint{
    int32_t x = 0;
    int32_t y = 0;
    int32_t t = 0;
    int32_t id = 0;

    inline void operator=(const PoseByIndex& e){
       x = e.x; 
       y = e.y; 
    }

    inline bool operator==(const PBIConstraint &e) const {
        if(e.x==x && e.y == y && e.t == t && e.id == id){
            return true;
        }else{
            return false;
        }
    }

    inline bool operator==(const PoseByIndex &e) const {
        if(e.x==x && e.y == y){
            return true;
        }else{
            return false;
        }
    }

};

struct LLNode{
    PoseByIndex pose;
    float fScore = 10000000.f;
    int32_t timestep = 0;
    int32_t rev_counter = 0;
    int32_t waiting_counter = 0;

    bool operator < (const dynamics::data::LLNode r) const {
        if(fScore < r.fScore){
            return false;
        }else{
            return true;
        }
    }
};

struct MLLNode{
    std::vector<VehicleID> vehicle_index; 
    MPoseByIndex poses;
    float fScore = 10000000.f;
    int32_t timestep = 0;
    int32_t rev_counter = 0;
    int32_t waiting_counter = 0;

    bool operator < (const dynamics::data::MLLNode r) const {
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
    float h = 0.f; 

    /* Velocity in m/s */
    float vel = 0.f;

    inline Pose2D  operator+(const Pose2D& e) const{
        return {pos + e.pos, e.h, e.vel};
    }
};

struct Pose2WithTime{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f}; 
    
    /* Heading in radians */
    float h = 0.f; 

    /* Velocity in cm/s */
    float vel = 0.f;

    float time_ms = 0.f;

    PoseByIndex baseNode;
    uint32_t path_depth_index = 0;
    uint32_t rem_seg_index = 0;

    inline void operator=(Pose2D e){
        pos = e.pos;
        h = e.h;
        vel = e.vel;
    }
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
    float s_acc = 0.f;
    float s_v = 0.f;

    float s_a_2 = 0.f;
    float s_acc_2 = 0.f;
    float s_v_2 = 0.f;

    bool is_acc_based = false;

    inline void operator=(const PoseByIndex& e){
       bi_pose.x = e.x; 
       bi_pose.y = e.y; 
       bi_pose.a = e.a;
       bi_pose.s = e.s;
    }

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
            assert(sizeof(std::size_t) == 64);
            std::size_t s_hash = static_cast<std::size_t>(p.s);
            assert(p.s < 4);
            std::size_t a_hash = static_cast<std::size_t>(p.a) << 2; // Shift by 2 bits (max s value is 4, needs 2 bits)
            assert(p.a < 15);
            std::size_t y_hash = static_cast<std::size_t>(p.y) << 6; // Shift by 6 bits (max a value is 15, needs 4 bits)
            assert(p.y < 127);
            std::size_t x_hash = static_cast<std::size_t>(p.x) << 13; // Shift by 13 bits (max y value is 127, needs 7 bits)
            assert(p.x < 127);
            std::size_t t_hash = static_cast<std::size_t>(p.t) << 20; // Shift by 20 bits (max x value is 127, needs 7 bits)
            assert(p.t < 2);

            return s_hash | a_hash | y_hash | x_hash | t_hash;
        }
    };

    template<> struct hash<dynamics::data::MPoseByIndex>
    {
       std::size_t operator()(const dynamics::data::MPoseByIndex& p) const noexcept
        {
            std::hash<dynamics::data::PoseByIndex> pbiHashFn;

            assert(sizeof(std::size_t) == 64);
            std::size_t hashv1 = static_cast<std::size_t>(pbiHashFn(p.vehicle_poses.at(0))); 
            std::size_t hashv2 = static_cast<std::size_t>(pbiHashFn(p.vehicle_poses.at(1))) << 21;
            std::size_t hashv3 = static_cast<std::size_t>(pbiHashFn(p.vehicle_poses.at(2))) << 42;

            return hashv1 | hashv2 | hashv3;
        }
    };
}

#endif //UTIL_POSE_HPP