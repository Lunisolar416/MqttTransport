#include "polygon_point_singleton.h"
#include <iostream>
#include <vector>

int main()
{
    std::cout << "=== 多边形点测试单例类演示 ===" << std::endl;

    // 获取单例实例
    auto tester = geometry::PolygonPointTester::GetInstance();
    
    // 或者使用便捷宏
    // auto tester = POLYGON_TESTER;

    // 创建一个简单的矩形多边形
    std::vector<Eigen::Vector2d> rectangle = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(10, 0),
        Eigen::Vector2d(10, 10),
        Eigen::Vector2d(0, 10)
    };

    // 测试点
    Eigen::Vector2d inside_point(5, 5);
    Eigen::Vector2d outside_point(15, 15);
    Eigen::Vector2d boundary_point(0, 5);

    std::cout << "\n1. 使用优化算法测试:" << std::endl;
    
    // 使用便捷方法
    bool is_inside = tester->IsPointInPolygon(rectangle, inside_point);
    bool is_outside = tester->IsPointInPolygon(rectangle, outside_point);
    bool is_on_boundary = tester->IsPointInPolygon(rectangle, boundary_point);

    std::cout << "点 (5, 5) 在多边形内: " << (is_inside ? "是" : "否") << std::endl;
    std::cout << "点 (15, 15) 在多边形内: " << (is_outside ? "是" : "否") << std::endl;
    std::cout << "点 (0, 5) 在多边形内: " << (is_on_boundary ? "是" : "否") << std::endl;

    std::cout << "\n2. 使用简单算法测试:" << std::endl;
    
    bool is_inside_simple = tester->IsPointInPolygonSimple(rectangle, inside_point);
    bool is_outside_simple = tester->IsPointInPolygonSimple(rectangle, outside_point);
    bool is_on_boundary_simple = tester->IsPointInPolygonSimple(rectangle, boundary_point);

    std::cout << "点 (5, 5) 在多边形内: " << (is_inside_simple ? "是" : "否") << std::endl;
    std::cout << "点 (15, 15) 在多边形内: " << (is_outside_simple ? "是" : "否") << std::endl;
    std::cout << "点 (0, 5) 在多边形内: " << (is_on_boundary_simple ? "是" : "否") << std::endl;

    std::cout << "\n3. 使用编译多边形测试:" << std::endl;
    
    // 编译多边形
    auto compiled_polygon = tester->CompilePolygon(rectangle, geometry::PointInPolygonAlgorithm::kOptimizedRayCasting);
    
    bool is_inside_compiled = tester->IsInOptimizedPolygon(compiled_polygon, inside_point.x(), inside_point.y());
    bool is_outside_compiled = tester->IsInOptimizedPolygon(compiled_polygon, outside_point.x(), outside_point.y());
    bool is_on_boundary_compiled = tester->IsInOptimizedPolygon(compiled_polygon, boundary_point.x(), boundary_point.y());

    std::cout << "点 (5, 5) 在多边形内: " << (is_inside_compiled ? "是" : "否") << std::endl;
    std::cout << "点 (15, 15) 在多边形内: " << (is_outside_compiled ? "是" : "否") << std::endl;
    std::cout << "点 (0, 5) 在多边形内: " << (is_on_boundary_compiled ? "是" : "否") << std::endl;

    std::cout << "\n4. 坐标转换测试:" << std::endl;
    
    // 经纬度坐标
    double lon = 116.4074; // 北京经度
    double lat = 39.9042;  // 北京纬度
    
    Eigen::Vector2d mercator_coord = tester->LonLatToMercator(lon, lat);
    std::cout << "北京坐标 (" << lon << ", " << lat << ") 转换为墨卡托投影: (" 
              << mercator_coord.x() << ", " << mercator_coord.y() << ")" << std::endl;

    // 多边形坐标转换
    std::vector<Eigen::Vector2d> lonlat_polygon = {
        Eigen::Vector2d(116.0, 39.0),
        Eigen::Vector2d(117.0, 39.0),
        Eigen::Vector2d(117.0, 40.0),
        Eigen::Vector2d(116.0, 40.0)
    };
    
    auto mercator_polygon = tester->ConvertPolygonLonLatToMercator(lonlat_polygon);
    std::cout << "多边形坐标转换完成，顶点数: " << mercator_polygon.size() << std::endl;

    std::cout << "\n5. 单例验证:" << std::endl;
    
    // 验证单例模式
    auto tester2 = geometry::PolygonPointTester::GetInstance();
    std::cout << "第一个实例地址: " << tester.get() << std::endl;
    std::cout << "第二个实例地址: " << tester2.get() << std::endl;
    std::cout << "是否为同一实例: " << (tester.get() == tester2.get() ? "是" : "否") << std::endl;

    std::cout << "\n=== 测试完成 ===" << std::endl;
    
    return 0;
} 