
#include <waypoint_server/route.h>

#include <fstream>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

#include <yaml-cpp/yaml.h>

namespace {
    static const std::string ns = "waypoint_server",
                             route_ns = "route",
                             count_of_skip_ids = "count_of_skip_ids",
                             ids = "ids";
}

namespace waypoint_server {
    Route::Route(const bool &debug) : debug(debug) {
        loop_mode.store(false);
        resetIndex();
    }

    const bool Route::skip(const unsigned int &count_of_ids) {
        count_of_skip_ids = count_of_ids;

        for(unsigned int i = 0; i < count_of_ids; i ++) {
            if(!forwardIndex()) {
                return FAILED;
            }
        }
        return SUCCESS;
    }

    void Route::loop(const bool &enable_loop) {
        loop_mode.store(enable_loop);
    }

    void Route::load(const std::string &filename) {
        std::lock_guard<std::mutex> lock(route_mutex);

        if(filename.empty()) {
            throw std::runtime_error("Load failed: empty filename waypoint_server::Route::load()");
        }
        std::ifstream input_file{filename};

        if(!input_file.is_open()) {
            std::cout
                << "Does not find "
                << filename
                << " waypoint_server::Route::load()"
                << std::endl;
            return;
        }
        YAML::Node yaml,
                   route_yaml;

        yaml = YAML::LoadFile(filename);
        route_yaml = yaml[::ns][::route_ns];

        route_indexes.clear();

        for(const auto &id : route_yaml[::ids]) {
            route_indexes.push_back(id.as<std::string>());
        }
        skip(route_yaml[::count_of_skip_ids].as<unsigned int>());

        if(!debug) {
            return;
        }
        std::cout << "ids:" << std::endl;

        for(const auto &id : route_indexes) {
            std::cout << "  " << id << std::endl;
        }
    }

    void Route::save(const std::string &filename) {
        std::lock_guard<std::mutex> lock(route_mutex);

        if(filename.empty()) {
            throw std::runtime_error("Save failed: empty filename waypoint_server::Route::save()");
        }
        YAML::Node yaml,
                   route_yaml;

        unsigned int count_of_index = 0;
        for(const auto &index : route_indexes) {
            route_yaml[::ids][count_of_index] = index;
            count_of_index ++;
        }
        route_yaml[::count_of_skip_ids] = count_of_skip_ids;

        yaml[::ns][::route_ns] = route_yaml;

        std::ofstream output_file{filename};

        if(!output_file.is_open()) {
            throw std::runtime_error("Failed open " + filename + " waypoint_server::Route::save()");
        }

        output_file << yaml << std::endl;
        output_file.close();
    }

    void Route::resetIndex() noexcept {
        is_finish.store(false);
        current_index.store(0);
    }

    const Map::Key &Route::getIndex() {
        std::lock_guard<std::mutex> lock(route_mutex);

        if(isEmpty()) {
            throw std::runtime_error("Get failed route indexes is zero waypoint_server::Route::getIndex()");
        }

        return route_indexes[current_index.load()];
    }

    const unsigned int &Route::getSkipIds() const {
        return count_of_skip_ids;
    }

    const bool Route::isEmpty() const {
        if(route_indexes.size() > 0) {
            return false;
        }
        return true;
    }

    const bool Route::isFinish() {
        return is_finish.load();
    }

    const bool Route::hasKey(const Map::Key &key) {
        bool has_key = false;

        for(const auto &i : route_indexes) {
            if(key == i) {
                has_key = true;
                break;
            }
        }
        if(has_key) {
            return true;
        }
        return false;
    }

    void Route::append(const Map::Key &key) {
        route_indexes.push_back(key);
    }

    void Route::erase() {
        std::lock_guard<std::mutex> lock(route_mutex);
        route_indexes.clear();
    }

    void Route::erase(const Map::Key &key) {
        std::lock_guard<std::mutex> lock(route_mutex);
        unsigned int erase_index = 0;

        for(
            auto itr = route_indexes.crbegin();
            itr != route_indexes.crend();
            ++ itr
        ) {
            if(*itr == key) {
                break;
            }
            erase_index ++;
        }
        if(route_indexes.size() > erase_index) {
            route_indexes.erase(
                route_indexes.begin()
                + (route_indexes.size() - erase_index - 1)
            );
        }
    }

    const bool Route::forwardIndex() {
        if(route_indexes.size() - 1 <= current_index.load()) {
            if(loop_mode.load()) {
                resetIndex();
                return SUCCESS;
            }
            is_finish.store(true);
            return SUCCESS;
        }
        current_index.store(current_index.load() + 1);
        return SUCCESS;
    }

    Route::Data &Route::data() {
        std::lock_guard<std::mutex> lock(route_mutex);

        return route_indexes;
    }

    Map::Key &Route::operator [] (const unsigned int &index) {
        std::lock_guard<std::mutex> lock(route_mutex);

        if(debug) {
            if(route_indexes.size() - 1 <= index) {
                throw std::out_of_range("Illigal access waypoint_server::Route::operator[]");
            }
        }
        return route_indexes[index];
    }
}

