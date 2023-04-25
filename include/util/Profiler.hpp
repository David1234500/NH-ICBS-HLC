#ifndef CBSProfiler_HPP
#define CBSProfiler_HPP

#include <chrono>
#include <iostream>
#include <fstream>
#include <map>
#include <mutex>
#include <stack>
#include <string>
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono;

enum Method {
    CBSBootstrap,
    CBSComplete,
    CBSWorkerIteration,
    AStarComplete,
    AStarIteration,
    CollisionDetect
};

class CBSProfiler {
public:
    static CBSProfiler& instance() {
        static CBSProfiler instance;
        return instance;
    }

    CBSProfiler(const CBSProfiler&) = delete;
    CBSProfiler& operator=(const CBSProfiler&) = delete;

    unsigned long long start(Method method) {
        // std::lock_guard<std::mutex> lock(m_mutex);

        // auto now = high_resolution_clock::now();
        // unsigned long long id = ++m_next_id;
        // m_starts[method][id] = now;
        // m_concurrent_calls[method]++;
        // m_info[method][id]["concurrent_calls"] = m_concurrent_calls[method];
        // m_info[method][id]["start_time"] = duration_cast<milliseconds>(now.time_since_epoch()).count();

        return 5;
    }

    void stop(Method method, unsigned long long id) {
        std::lock_guard<std::mutex> lock(m_mutex);

        // auto now = high_resolution_clock::now();
        // auto start_time = m_starts[method][id];
        // m_starts[method].erase(id);

        // duration<double> elapsed_time = now - start_time;
        // m_info[method][id]["end_time"] = duration_cast<milliseconds>(now.time_since_epoch()).count();
        // m_info[method][id]["duration"] = duration_cast<milliseconds>(elapsed_time).count();
        // m_concurrent_calls[method]--;
    }

    void export_to_json(const std::string &filename) {
    // json j;
    // for (int i = 0; i < 6; ++i) {
    //     Method method = static_cast<Method>(i);

    //     for (const auto& [id, call_info] : m_info[method]) {
    //         if (m_starts[method].count(id) > 0) {
    //             m_info[method][id]["incomplete"] = true;
    //         }
    //         j[method_to_string(method)][std::to_string(id)] = m_info[method][id];
    //     }
    // }
    // std::ofstream file(filename);
    // if (file.is_open()) {
    //     file << j.dump(4);
    //     file.close();
    // } else {
    //     std::cerr << "Unable to open file " << filename << std::endl;
    // }
}


private:
    CBSProfiler() = default;

    std::string method_to_string(Method method) {
        switch (method) {
            case CBSBootstrap: return "CBSBootstrap";
            case CBSComplete: return "CBSComplete";
            case CBSWorkerIteration: return "CBSWorkerIteration";
            case AStarComplete: return "AStarComplete";
            case AStarIteration: return "AStarIteration";
            case CollisionDetect: return "CollisionDetect";
            default: return "Unknown";
        }
    }

    std::mutex m_mutex;
    unsigned long long m_next_id = 0;
    std::map<Method, std::map<unsigned long long, time_point<high_resolution_clock>>> m_starts;
    std::map<Method, std::map<unsigned long long, json>> m_info;
    std::map<Method, int> m_concurrent_calls;
};

#endif