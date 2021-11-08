
#include <waypoint_server/map.h>

#include <fstream>
#include <stdexcept>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

namespace {
    // YAML node keys
    static const std::string ns = "waypoint_server",
                             waypoints_ns = "waypoints",
                             scale = "scale",
                             position = "position",
                             euler_angles = "euler_angles",
                             x = "x",
                             y = "y",
                             z = "z",
                             id = "id",
                             connections = "connections",
                             properties = "properties",
                             key = "key",
                             value = "value";
}

namespace waypoint_server {
    void geometricConverter(Map &),
         geometricDeconverter(Map &);

    Map::Map(const bool &debug) : debug(debug) {
        position = Eigen::Vector3f::Zero();
        euler_angles = Eigen::Vector3f::Zero();

        scale = 1.0;

        is_converted = false;
    }

    void Map::load(const std::string &filename) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);

        if(filename.empty()) {
            throw std::runtime_error("Load failed: empty waypoint_server::Map::load()");
        }
        std::ifstream input_file{filename};

        if(!input_file.is_open()) {
            std::cout
                << "Does not find "
                << filename
                << " waypoint_server::Map::load()"
                << std::endl;
            return;
        }

        YAML::Node yaml,
                   waypoints_yaml;

        yaml = YAML::LoadFile(filename);

        named_waypoints.clear();

        scale = yaml[::ns][::scale].as<float>();
        position.x() = yaml[::ns][::position][::x].as<float>();
        position.y() = yaml[::ns][::position][::y].as<float>();
        position.z() = yaml[::ns][::position][::z].as<float>();
        euler_angles.x() = yaml[::ns][::euler_angles][::x].as<float>();
        euler_angles.y() = yaml[::ns][::euler_angles][::y].as<float>();
        euler_angles.z() = yaml[::ns][::euler_angles][::z].as<float>();

        waypoints_yaml = yaml[::ns][::waypoints_ns];

        for(unsigned int i = 0; i < waypoints_yaml.size(); i ++) {
            YAML::Node data = waypoints_yaml[i];

            if(data[::id].IsNull()) {
                continue;
            }
            std::string index = data[::id].as<std::string>();

            if(data[::position].IsNull()) {
                named_waypoints[index].enable_position = false;
            }
            else {
                named_waypoints[index].goal(0) = data[::position][::x].as<float>();
                named_waypoints[index].goal(1) = data[::position][::y].as<float>();
                named_waypoints[index].goal(2) = data[::position][::z].as<float>();
            }

            if(data[::euler_angles].IsNull()) {
                named_waypoints[index].enable_orientation = false;
            }
            else {
                named_waypoints[index].goal(3) = data[::euler_angles][::x].as<float>();
                named_waypoints[index].goal(4) = data[::euler_angles][::y].as<float>();
                named_waypoints[index].goal(5) = data[::euler_angles][::z].as<float>();
            }

            if(!data[::connections].IsNull()) {
                for(unsigned int i = 0; i < data[::connections].size(); i ++) {
                    named_waypoints[index].connections.push_back(data[::connections][i].as<std::string>());
                }
            }
            if(!data[::properties].IsNull()) {
                for(const auto &propertiy : data[::properties]) {
                    named_waypoints[index].properties[propertiy[::key].as<std::string>()]
                        = propertiy[::value].as<std::string>();
                }
            }
        }

        setConvertStatus(false);
        geometricConverter(*this);

        if(!debug) {
            return;
        }
        std::cout << "scale: " << scale << std::endl;
        std::cout << "position: " << position.transpose() << std::endl;
        std::cout << "euler_angles: " << euler_angles.transpose() << std::endl;
        for(const auto &[key, waypoint] : named_waypoints) {
            std::cout << "id: " << key << std::endl;
            std::cout << "  goal: " << waypoint.goal.transpose() << std::endl;
            std::cout << "  enable_position: " << std::boolalpha << waypoint.enable_position << std::endl;
            std::cout << "  enable_orientation: " << std::boolalpha << waypoint.enable_orientation << std::endl;

            for(const auto &connect : waypoint.connections) {
                std::cout << "  connect: " << connect << std::endl;
            }
            for(const auto &[key, value] : waypoint.properties) {
                std::cout << "  key: " << key << ", value: " << value << std::endl;
            }
        }
    }

    void Map::save(const std::string &filename) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);

        if(filename.empty()) {
            throw std::runtime_error("Save failed: empty filename waypoint_server::Map::save()");
        }
        YAML::Node yaml,
                   waypoints_yaml;

        geometricDeconverter(*this);

        waypoints_yaml[::scale] = scale;
        waypoints_yaml[::position][::x] = position.x();
        waypoints_yaml[::position][::y] = position.y();
        waypoints_yaml[::position][::z] = position.z();
        waypoints_yaml[::euler_angles][::x] = euler_angles.x();
        waypoints_yaml[::euler_angles][::y] = euler_angles.y();
        waypoints_yaml[::euler_angles][::z] = euler_angles.z();

        unsigned int count_of_waypoints = 0;

        for(const auto &[key, data] : named_waypoints) {
            YAML::Node waypoint_yaml;

            waypoint_yaml[::id] = key;
            waypoint_yaml[::position][::x] = data.goal(0);
            waypoint_yaml[::position][::y] = data.goal(1);
            waypoint_yaml[::position][::z] = data.goal(2);
            waypoint_yaml[::euler_angles][::x] = data.goal(3);
            waypoint_yaml[::euler_angles][::y] = data.goal(4);
            waypoint_yaml[::euler_angles][::z] = data.goal(5);

            unsigned int count_of_properties = 0;

            for(const auto &[pkey, pdata] : data.properties) {
                YAML::Node properties_yaml;

                properties_yaml[::key] = pkey;
                properties_yaml[::value] = pdata;

                waypoint_yaml[::properties][count_of_properties] = properties_yaml;
                count_of_properties ++;
            }
            unsigned int count_of_connections = 0;

            for(const auto &connect : data.connections) {
                waypoint_yaml[::connections][count_of_connections] = connect;
                count_of_connections ++;
            }

            waypoints_yaml[::waypoints_ns][count_of_waypoints] = waypoint_yaml;
            count_of_waypoints ++;
        }
        yaml[::ns] = waypoints_yaml;

        std::ofstream output_file{filename};

        if(!output_file.is_open()) {
            throw std::runtime_error("Failed open " + filename + " waypoint_server::Map::save()");
        }
        output_file << yaml << std::endl;;
        output_file.close();
    }

    Map::NamedWaypoints &Map::data() noexcept {
        return named_waypoints;
    }

    void Map::setPosition(const Eigen::Vector3f &position) {
        this->position = position;
    }

    void Map::setEulerAngle(const Eigen::Vector3f &euler_angles) {
        this->euler_angles = euler_angles;
    }

    const float &Map::getScale() const {
        return scale;
    }

    const Eigen::Vector3f &Map::getPosition() const {
        return position;
    }

    const Eigen::Vector3f &Map::getEulerAngle() const {
        return euler_angles;
    }

    void Map::setQuaternion(const Key &key, const Waypoint::Quaternion &quaternion) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);

        named_waypoints[key].quaternion = quaternion;
        named_waypoints[key].goal.block(3, 0, 3, 1) = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    }

    void Map::setQuaternion(const Key &key) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);

        named_waypoints[key].goal.block(3, 0, 3, 1) = named_waypoints[key].quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    }

    void Map::updateQuaternion(const Key &key) {
        if(named_waypoints.count(key) == 0) {
            throw std::runtime_error(
                "Update failed because not found waypoint waypoint_server::Map::updateQuaternion()"
            );
        }
        Waypoint::Quaternion quaternion;
        const Eigen::Vector3f waypoint_euler_angles{named_waypoints[key].goal.block(3, 0, 3, 1)};

        quaternion = Eigen::AngleAxisf(waypoint_euler_angles.z(), waypoint_euler_angles.UnitZ());
        quaternion = Eigen::AngleAxisf(waypoint_euler_angles.y(), waypoint_euler_angles.UnitY()) * quaternion;
        quaternion = Eigen::AngleAxisf(waypoint_euler_angles.x(), waypoint_euler_angles.UnitX()) * quaternion;

        std::lock_guard<std::mutex> lock(waypoints_mutex);

        named_waypoints[key].quaternion = quaternion;
    }

    const bool &Map::isConverted() const {
        return is_converted;
    }

    void Map::setConvertStatus(const bool &status) {
        is_converted = status;
    }

    void Map::append(
        const Key &key,
        const Eigen::Vector3f &position,
        const Eigen::Quaternionf &quaternion
    ) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);
        named_waypoints[key].goal.block(0, 0, 3, 1) = position;
        setQuaternion(key, quaternion);
    }

    void Map::erase() {
        std::lock_guard<std::mutex> lock(waypoints_mutex);
        named_waypoints.clear();
    }

    void Map::erase(const Key &key) {
        if(!hasKey(key)) {
            return;
        }
        std::lock_guard<std::mutex> lock(waypoints_mutex);
        named_waypoints.erase(key);
    }

    const bool Map::hasKey(const Key &key) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);

        if(named_waypoints.count(key) == 0) {
            return false;
        }
        return true;
    }

    Waypoint &Map::operator [] (const Key &key) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);

        return named_waypoints[key];
    }

    void geometricConverter(Map &map) {
        if(map.isConverted()) {
            return;
        }
        if(map.debug) {
            std::cout << "Start geometricConverter" << std::endl;
        }

        Eigen::Vector3f map_position,
                        map_euler_angles;

        map_position = map.getPosition();
        map_euler_angles = map.getEulerAngle();

        Waypoint::Quaternion map_quaternion;

        map_quaternion = Eigen::AngleAxisf(map_euler_angles.z(), map_euler_angles.UnitZ());
        map_quaternion = Eigen::AngleAxisf(map_euler_angles.y(), map_euler_angles.UnitY()) * map_quaternion;
        map_quaternion = Eigen::AngleAxisf(map_euler_angles.x(), map_euler_angles.UnitX()) * map_quaternion;

        for(auto &&[key, waypoint] : map.data()) {
            Eigen::Vector3f position = waypoint.goal.block(0, 0, 3, 1);
            Eigen::Vector3f euler_angles = waypoint.goal.block(3, 0, 3, 1);

            Waypoint::Quaternion quaternion;

            quaternion = Eigen::AngleAxisf(euler_angles.z(), euler_angles.UnitZ());
            quaternion = Eigen::AngleAxisf(euler_angles.y(), euler_angles.UnitY()) * quaternion;
            quaternion = Eigen::AngleAxisf(euler_angles.x(), euler_angles.UnitX()) * quaternion;

            quaternion = quaternion * map_quaternion;
            waypoint.quaternion = quaternion;
            position = map_quaternion * (position + map_position);
            position = position * map.getScale();

            waypoint.goal.block(0, 0, 3, 1) = position;
            waypoint.goal.block(3, 0, 3, 1) = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
            waypoint.quaternion = quaternion;
        }

        map.setConvertStatus(true);
    }

    void geometricDeconverter(Map &map) {
        if(!map.isConverted()) {
            return;
        }
        if(map.debug) {
            std::cout << "Start geometricDeconverter" << std::endl;
        }

        Eigen::Vector3f map_position,
                        map_euler_angles;

        map_position = map.getPosition();
        map_euler_angles = map.getEulerAngle();

        Waypoint::Quaternion map_quaternion;

        map_quaternion = Eigen::AngleAxisf(map_euler_angles.z(), map_euler_angles.UnitZ());
        map_quaternion = Eigen::AngleAxisf(map_euler_angles.y(), map_euler_angles.UnitY()) * map_quaternion;
        map_quaternion = Eigen::AngleAxisf(map_euler_angles.x(), map_euler_angles.UnitX()) * map_quaternion;

        for(auto &&[key, waypoint] : map.data()) {
            Eigen::Vector3f position = waypoint.goal.block(0, 0, 3, 1);
            Eigen::Vector3f euler_angles = waypoint.goal.block(3, 0, 3, 1);

            Waypoint::Quaternion quaternion;

            quaternion = Eigen::AngleAxisf(euler_angles.z(), euler_angles.UnitZ());
            quaternion = Eigen::AngleAxisf(euler_angles.y(), euler_angles.UnitY()) * quaternion;
            quaternion = Eigen::AngleAxisf(euler_angles.x(), euler_angles.UnitX()) * quaternion;
            quaternion = quaternion * map_quaternion.inverse();

            position = position / map.getScale();
            position = map_quaternion.inverse() * position - map_position;

            waypoint.goal.block(0, 0, 3, 1) = position;
            waypoint.goal.block(3, 0, 3, 1) = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
            waypoint.quaternion = quaternion;
        }

        map.setConvertStatus(false);
    }
}

