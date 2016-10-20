#include "Common.h"

const std::string Constants::LeftWindowName = "Left";
const std::string Constants::RightWindowName = "Right";
const std::string Constants::MatchesWindowName = "Matches";

const std::chrono::duration<long long, std::milli> Constants::CAPTURE_FROM_CAMERA_INTERVAL = std::chrono::milliseconds(10);
// from specs camera angle is 42 degree, so half of it 21
const double Constants::CAMERA_ANGLE_WIDTH_MULTIPLIER = 0.38386403503541579597144840810327; // tan 21
const double Constants::CLOSEST_DETECTABLE_RANGE = 0.33; //in seconds [that should be multiplied by speed to get distance]
const double Constants::FARTHEST_DETECTABLE_RANGE = 0.7; //in seconds [that should be multiplied by speed to get distance]
const double Constants::OPTICAL_AXIS_DISTANCE = 60.0; // mm
const double Constants::CHASSIS_WIDTH = 130.0; // mm
const double Constants::WHEEL_WIDTH = 20.0; // mm
const double Constants::OBSTACLE_LENGTH = 250.0; // mm
const double Constants::OBSTACLE_DISTANCE = 100.0; // mm
