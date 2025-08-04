#pragma once
#include <cmath>

class GeoUtils
{
  public:
    // 返回单位：米
    static double computeDistance(double lon1_deg, double lat1_deg, double lon2_deg, double lat2_deg);

    // 起点指向终点的方位角（单位：度，范围 [0, 360)）
    static double computeBearing(double lon1_deg, double lat1_deg, double lon2_deg, double lat2_deg);
    // 计算两个角度的相对角度，范围 [-180, 180]
    static double computeRelBearing(double targetAngle_deg, double selfAngle_deg);
};
