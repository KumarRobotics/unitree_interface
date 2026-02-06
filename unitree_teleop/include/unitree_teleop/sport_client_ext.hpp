#ifndef UNITREE_TELEOP_SPORT_CLIENT_EXT_HPP_
#define UNITREE_TELEOP_SPORT_CLIENT_EXT_HPP_

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/sport/sport_api.hpp>
#include <unitree/robot/go2/public/jsonize_type.hpp>

// SetGait/SwitchGait API ID (1011) - not in the default go2 sport_api.hpp
const int32_t ROBOT_SPORT_API_ID_SETGAIT = 1011;

class SportClientExt : public unitree::robot::go2::SportClient
{
public:
    explicit SportClientExt(bool enableLease = false)
        : unitree::robot::go2::SportClient(enableLease) {}

    ~SportClientExt() = default;
    
    int32_t SetGait(int gait_type)
    {
        std::string parameter, data;
        unitree::robot::go2::JsonizeDataInt json;
        json.data = gait_type;
        parameter = unitree::common::ToJsonString(json);
        return Call(ROBOT_SPORT_API_ID_SETGAIT, parameter, data);
    }
};

#endif // UNITREE_TELEOP_SPORT_CLIENT_EXT_HPP_
