#include "polygon_point_singleton.h"
#include <algorithm>
#include <cmath>

namespace geometry
{

bool PolygonPointTester::IsPointOnSegment(double x, double y, const Eigen::Vector2d &a, const Eigen::Vector2d &b,
                                          double eps)
{
    const double cross = (b.x() - a.x()) * (y - a.y()) - (b.y() - a.y()) * (x - a.x());
    if (std::abs(cross) > eps)
        return false;
    const double dot = (x - a.x()) * (x - b.x()) + (y - a.y()) * (y - b.y());
    return dot <= eps;
}

bool PolygonPointTester::IsInOptimizedPolygon(const CompiledPolygon &polygon, double x, double y, bool include_boundary,
                                              bool use_bbox_precheck)
{
    // 退化多边形（所有点共线）面积为零，所以没有点能在它们内部
    if (polygon.is_degenerate)
    {
        return false;
    }

    if (use_bbox_precheck && !polygon.bounding_box.contains(Eigen::Vector2d(x, y)))
    {
        return false;
    }

    // 检查点是否在边界上
    bool on_boundary = false;
    for (size_t i = 0; i < polygon.vertices.size(); ++i)
    {
        if (IsPointOnSegment(x, y, polygon.vertices[i], polygon.vertices[(i + 1) % polygon.vertices.size()]))
        {
            on_boundary = true;
            break;
        }
    }

    // 如果在边界上，根据include_boundary标志返回
    if (on_boundary)
    {
        return include_boundary;
    }

    // 否则，对内部点使用射线投射
    bool inside = false;
    for (const auto &e : polygon.edges)
    {
        if (e.horizontal)
            continue;
        if (y >= e.ymin && y < e.ymax)
        {
            if ((y - e.y1) * e.inv_slope + e.x1 > x)
            {
                inside = !inside;
            }
        }
    }
    return inside;
}

bool PolygonPointTester::IsInSimplePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point,
                                           bool include_boundary, bool use_bbox_precheck)
{
    // 计算边界框用于预检查
    Eigen::AlignedBox2d bbox;
    bbox.setEmpty();
    for (const auto &p : polygon)
        bbox.extend(p);

    if (use_bbox_precheck && !bbox.contains(point))
    {
        return false;
    }

    // 使用显式边界检查以保持算法一致性
    bool on_boundary = false;
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        if (IsPointOnSegment(point.x(), point.y(), polygon[i], polygon[(i + 1) % polygon.size()]))
        {
            on_boundary = true;
            break;
        }
    }
    if (on_boundary)
    {
        return include_boundary;
    }

    // 对内部点使用射线投射逻辑
    bool is_inside = false;
    int n_vertices = static_cast<int>(polygon.size());
    if (n_vertices < 3)
        return false;
    for (int i = 0, j = n_vertices - 1; i < n_vertices; j = i++)
    {
        const auto &vertex_i = polygon[i];
        const auto &vertex_j = polygon[j];
        if ((vertex_i.y() > point.y()) != (vertex_j.y() > point.y()))
        {
            double x_intersection =
                (vertex_j.x() - vertex_i.x()) * (point.y() - vertex_i.y()) / (vertex_j.y() - vertex_i.y()) +
                vertex_i.x();
            if (point.x() < x_intersection)
            {
                is_inside = !is_inside;
            }
        }
    }
    return is_inside;
}

CompiledPolygon PolygonPointTester::CompilePolygon(const std::vector<Eigen::Vector2d> &vertices,
                                                   PointInPolygonAlgorithm algorithm)
{
    CompiledPolygon result;

    // 清理顶点，移除重复
    result.vertices = SanitizePolygonVertices(vertices);

    if (result.vertices.size() < 3)
    {
        result.is_degenerate = true;
        return result;
    }

    // 计算边界框
    result.bounding_box.setEmpty();
    for (const auto &v : result.vertices)
        result.bounding_box.extend(v);

    // 检查多边形是否退化（所有点共线）
    // 使用多边形面积的鲁棒退化检查，具有尺度感知的epsilon
    const double area2 = std::abs(TwiceSignedArea(result.vertices));
    const double diag_sq = (result.bounding_box.max() - result.bounding_box.min()).squaredNorm();
    const double area_eps = std::max(1.0, diag_sq) * 1e-12;
    result.is_degenerate = (area2 < area_eps);

    // 如果使用优化算法，预计算边数据
    if (algorithm == PointInPolygonAlgorithm::kOptimizedRayCasting)
    {
        const size_t n = result.vertices.size();
        result.edges.reserve(n);
        for (size_t i = 0; i < n; ++i)
        {
            const auto &a = result.vertices[i];
            const auto &b = result.vertices[(i + 1) % n];
            PolygonEdge e;
            e.x1 = a.x();
            e.y1 = a.y();
            e.x2 = b.x();
            e.y2 = b.y();
            e.horizontal = std::abs(e.y2 - e.y1) < 1e-12;
            if (!e.horizontal)
            {
                e.inv_slope = (e.x2 - e.x1) / (e.y2 - e.y1);
                if (e.y1 < e.y2)
                {
                    e.ymin = e.y1;
                    e.ymax = e.y2;
                }
                else
                {
                    e.ymin = e.y2;
                    e.ymax = e.y1;
                }
            }
            else
            {
                e.inv_slope = 0.0;
                e.ymin = e.ymax = e.y1;
            }
            result.edges.push_back(e);
        }
    }

    return result;
}

std::vector<Eigen::Vector2d> PolygonPointTester::SanitizePolygonVertices(const std::vector<Eigen::Vector2d> &vertices,
                                                                         double epsilon)
{
    if (vertices.size() < 2)
    {
        return vertices;
    }

    std::vector<Eigen::Vector2d> result;
    result.reserve(vertices.size());
    result.push_back(vertices.front());

    for (size_t i = 1; i < vertices.size(); ++i)
    {
        if ((vertices[i] - result.back()).norm() > epsilon)
        {
            result.push_back(vertices[i]);
        }
    }

    // 最终检查：如果最后一个点是第一个点的重复，则移除它
    if (result.size() > 2 && (result.back() - result.front()).norm() < epsilon)
    {
        result.pop_back();
    }

    return result;
}

double PolygonPointTester::TwiceSignedArea(const std::vector<Eigen::Vector2d> &vertices)
{
    const size_t n = vertices.size();
    if (n < 3)
        return 0.0;
    double a = 0.0;
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        a += Cross2(vertices[j], vertices[i]);
    }
    return a; // >0 表示逆时针，<0 表示顺时针（假设y轴向上）
}

Eigen::Vector2d PolygonPointTester::LonLatToMercator(double lon_deg, double lat_deg)
{
    constexpr double R = 6378137.0; // 地球半径，单位米 (WGS84标准)

    double x = R * lon_deg * M_PI / 180.0;

    // 限制纬度范围，避免超过投影极限
    if (lat_deg > 89.5)
        lat_deg = 89.5;
    if (lat_deg < -89.5)
        lat_deg = -89.5;

    double lat_rad = lat_deg * M_PI / 180.0;
    double y = R * std::log(std::tan(M_PI / 4.0 + lat_rad / 2.0));

    return Eigen::Vector2d(x, y);
}

std::vector<Eigen::Vector2d> PolygonPointTester::ConvertPolygonLonLatToMercator(
    const std::vector<Eigen::Vector2d> &lonlat_points)
{
    std::vector<Eigen::Vector2d> projected;
    projected.reserve(lonlat_points.size());
    for (const auto &p : lonlat_points)
    {
        // 传入 p.x()是经度，p.y()是纬度
        projected.push_back(LonLatToMercator(p.x(), p.y()));
    }
    return projected;
}

bool PolygonPointTester::IsPointInPolygon(const std::vector<Eigen::Vector2d> &vertices, const Eigen::Vector2d &point,
                                          bool include_boundary)
{
    // 使用优化算法
    CompiledPolygon compiled = CompilePolygon(vertices, PointInPolygonAlgorithm::kOptimizedRayCasting);
    return IsInOptimizedPolygon(compiled, point.x(), point.y(), include_boundary);
}

bool PolygonPointTester::IsPointInPolygonSimple(const std::vector<Eigen::Vector2d> &vertices,
                                                const Eigen::Vector2d &point, bool include_boundary)
{
    // 使用简单算法
    return IsInSimplePolygon(vertices, point, include_boundary);
}

bool PolygonPointTester::LoadPolygon(const std::vector<Eigen::Vector2d> &lonlat_vertices)
{
    if (lonlat_vertices.size() < 3)
    {
        std::cerr << "LoadPolygon: polygon has fewer than 3 points\n";
        polygon_valid = false;
        return false;
    }

    // 转为墨卡托投影
    auto polygon_mercator = ConvertPolygonLonLatToMercator(lonlat_vertices);

    // 编译多边形
    compiled_polygon = CompilePolygon(polygon_mercator, PointInPolygonAlgorithm::kOptimizedRayCasting);

    // 设置有效标记
    polygon_valid = !compiled_polygon.is_degenerate;

    return polygon_valid;
}

bool PolygonPointTester::isPointInPolygon(const Eigen::Vector2d &point)
{
    if (!polygon_valid)
    {
        std::cerr << "isPointInPolygon: polygon is not valid\n";
        return false;
    }

    // 转投影
    auto point_mercator = LonLatToMercator(point.x(), point.y());
    return IsInOptimizedPolygon(compiled_polygon, point_mercator.x(), point_mercator.y(), true, true);
}
} // namespace geometry