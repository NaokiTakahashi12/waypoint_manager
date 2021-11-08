
#pragma once

#include <unordered_map>
#include <mutex>

#include <Eigen/Dense>

#include "waypoint.h"

namespace waypoint_server {
    class Map {
        public :
            using Key = std::string;
            using NamedWaypoints = std::unordered_map<Key, Waypoint>;

            Map(const bool &debug = false);

            void load(const std::string &filename);
            void save(const std::string &dilename);

            NamedWaypoints &data() noexcept;

            void setPosition(const Eigen::Vector3f &);
            void setEulerAngle(const Eigen::Vector3f &);

            const float &getScale() const;
            const Eigen::Vector3f &getPosition() const;
            const Eigen::Vector3f &getEulerAngle() const;

            void setQuaternion(const Key &, const Waypoint::Quaternion &);
            void setQuaternion(const Key &);
            void updateQuaternion(const Key &);

            const bool &isConverted() const;
            void setConvertStatus(const bool &status);

            void append(
                const Key &,
                const Eigen::Vector3f &position,
                const Eigen::Quaternionf &quaternion
            );

            void erase();
            void erase(const Key &);

            const bool hasKey(const Key &);

            Waypoint &operator [] (const Key &);

            const bool debug;

        private :
            bool is_converted;

            float scale;

            std::mutex waypoints_mutex;

            Eigen::Vector3f position,
                            euler_angles;

            NamedWaypoints named_waypoints;
    };
}

