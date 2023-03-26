#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Config {
public:
    static Config& getInstance(const std::string& filename = "../config.json") {
        static Config instance(filename);
        return instance;
    }

    Config(Config const&) = delete;
    void operator=(Config const&) = delete;

    template <typename T>
    T get(const std::initializer_list<std::string>& keys) const {
        try {
            json temp = config_data;
            for (const std::string& key : keys) {
                temp = temp.at(key);
            }
            return temp.get<T>();
        } catch (const json::out_of_range& e) {
            for (const std::string& key : keys) {
              std::cout << "Missing key " << key << std::endl;
            }
            throw std::runtime_error("Config key not found");
        }
    }

    json getConfigData() const {
        return config_data;
    }

    int32_t getXstep(){
        double xcm = get<double>({"map","x_cm"});
        double dstep = get<double>({"disc","dstep"});
        return round(xcm/dstep);
    }

    int32_t getYstep(){
        double xcm = get<double>({"map","y_cm"});
        double dstep = get<double>({"disc","dstep"});
        return round(xcm/dstep);
    }

private:
    Config(const std::string& filename) {
        if (!filename.empty()) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("Unable to open config file: " + filename);
            }
            file >> config_data;
        }
    }

    json config_data;
};

#endif