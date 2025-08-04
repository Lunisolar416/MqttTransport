#include "GeoUtils.h"
#include <cmath>
static constexpr double EARTH_RADIUS = 6371000.0; // 单位：米
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
double GeoUtils::computeDistance(double lon1_deg, double lat1_deg, double lon2_deg, double lat2_deg)
{
    double lat1 = lat1_deg * DEG_TO_RAD;
    double lon1 = lon1_deg * DEG_TO_RAD;
    double lat2 = lat2_deg * DEG_TO_RAD;
    double lon2 = lon2_deg * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::pow(std::sin(dlat / 2), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2), 2);

    double c = 2 * std::asin(std::sqrt(a));
    return EARTH_RADIUS * c;
}

double GeoUtils::computeBearing(double lon1_deg, double lat1_deg, double lon2_deg, double lat2_deg)
{
    double lat1 = lat1_deg * DEG_TO_RAD;
    double lat2 = lat2_deg * DEG_TO_RAD;
    double dlon = (lon2_deg - lon1_deg) * DEG_TO_RAD;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double initial_bearing_rad = std::atan2(x, y);
    double initial_bearing_deg = fmod((initial_bearing_rad * RAD_TO_DEG + 360.0), 360.0);
    return initial_bearing_deg;
}

double GeoUtils::computeRelBearing(double targetAngle_deg, double selfAngle_deg)
{
    double diff = targetAngle_deg - selfAngle_deg;
    diff = fmod(diff, 360.0);
    if (diff < 0)
        diff += 360.0;
    return diff;
}
