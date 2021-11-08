
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "map.h"

namespace waypoint_server {
    class Route {
        public :
            using Data = std::vector<Map::Key>;

            static constexpr bool SUCCESS = true,
                                  FAILED = false;

            Route(const bool &debug = false);

            const bool skip(const unsigned int &count_of_ids);
            void loop(const bool &);

            void load(const std::string &filename);
            void save(const std::string &filename);

            void resetIndex() noexcept;
            const bool forwardIndex();
            const Map::Key &getIndex();

            const unsigned int &getSkipIds() const;

            const bool isEmpty() const;
            const bool isFinish();

            const bool hasKey(const Map::Key &);

            void append(const Map::Key &);

            void erase();
            void erase(const Map::Key &);

            Data &data();

            Map::Key &operator [] (const unsigned int &index);

            const bool debug;

        private :
            std::atomic<bool> loop_mode;
            std::atomic<bool> is_finish;
            std::atomic<unsigned int> current_index;

            unsigned int count_of_skip_ids;

            std::mutex route_mutex;

            Data route_indexes;
    };
}

