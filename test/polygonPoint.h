#ifndef POLYGONPOINT_H
#define POLYGONPOINT_H

#include <Eigen/Dense>
#include <vector>

namespace geometry
{

/// \brief 判断点是否在多边形内的算法枚举
enum class PointInPolygonAlgorithm
{
    /// \brief 简单射线投射算法
    /// \details 直接实现射线投射算法，计算从测试点发出的水平射线与多边形边的交点数
    ///          奇数个交点表示在多边形内部，偶数个表示在外部
    ///          时间复杂度: O(n)，其中n是顶点数
    ///          空间复杂度: O(1)
    ///          边界处理: 基础，可能存在浮点数精度问题
    kSimpleRayCasting,

    /// \brief 优化的射线投射算法，预计算边数据
    /// \details 增强的射线投射算法，在多边形编译时预计算边属性以提高运行时性能
    ///          特性包括:
    ///          - 预计算的边斜率和边界
    ///          - 显式边界检测，可配置包含性
    ///          - 退化多边形检测（共线点）
    ///          - 边界框预检查优化
    ///          时间复杂度: O(n) 每次查询，O(n) 预处理
    ///          空间复杂度: O(n) 用于边数据存储
    kOptimizedRayCasting
};

/// \brief 预计算的边数据，用于优化的射线投射算法
/// \details 存储多边形边的预计算属性，以加速点-多边形测试
struct PolygonEdge
{
    double x1, y1;     ///< 边的起点坐标
    double x2, y2;     ///< 边的终点坐标
    double ymin, ymax; ///< 边的垂直边界，用于快速拒绝
    double inv_slope;  ///< 预计算的逆斜率: (x2-x1)/(y2-y1) 用于非水平边
    bool horizontal;   ///< 标识边是否为水平（|y2-y1| < 1e-12）
};

/// \brief 编译后的多边形数据结构，用于高效的地理围栏测试
/// \details 包含原始顶点和预计算的数据结构，加速点-多边形查询
struct CompiledPolygon
{
    /// \brief 原始多边形顶点，逆时针顺序
    std::vector<Eigen::Vector2d> vertices;

    /// \brief 预计算的边数据，用于优化的射线投射
    std::vector<PolygonEdge> edges;

    /// \brief 轴对齐边界框，用于快速拒绝测试
    /// \details 边界框外的点保证在多边形外部
    Eigen::AlignedBox2d bounding_box;

    /// \brief 标识所有顶点是否共线
    /// \details 退化的多边形面积为零，不能包含任何点
    ///          使用叉积测试检测，epsilon = 1e-9
    bool is_degenerate = false;
};

/// \brief 检查点是否精确位于线段上
/// \details 使用叉积检查共线性，使用点积检查线段边界
/// \param[in] x 测试点的x坐标
/// \param[in] y 测试点的y坐标
/// \param[in] a 线段的起点
/// \param[in] b 线段的终点
/// \param[in] eps 浮点数比较的容差（默认: 1e-9）
/// \return 如果点在容差范围内位于线段上则为true
bool IsPointOnSegment(double x, double y, const Eigen::Vector2d &a, const Eigen::Vector2d &b, double eps = 1e-9);

/// \brief 测试点是否在预编译的多边形内（优化算法）
/// \details 实现优化的射线投射，具有显式边界处理:
///          1. 使用边界框快速拒绝（如果启用）
///          2. 使用IsPointOnSegment()进行显式边界检测
///          3. 使用预计算的边斜率进行射线投射内部测试
/// \param[in] polygon 预编译的多边形，包含边数据
/// \param[in] x 测试点的x坐标（与多边形相同的坐标系）
/// \param[in] y 测试点的y坐标（与多边形相同的坐标系）
/// \param[in] include_boundary 边界点是否被视为"内部"
/// \param[in] use_bbox_precheck 是否使用边界框预检查
/// \return 如果点在内部（或在边界上且include_boundary=true）则为true
/// \return 如果点在外部或多边形退化则为false
/// \note 时间复杂度: O(n)，其中n是边数
bool IsInOptimizedPolygon(const CompiledPolygon &polygon, double x, double y, bool include_boundary = true,
                          bool use_bbox_precheck = true);

/// \brief 测试点是否在原始多边形内（简单算法）
/// \details 实现基础射线投射算法:
///          - 从测试点向+无穷大投射水平射线
///          - 计算与多边形边的交点数
///          - 奇数个交点 = 内部，偶数个交点 = 外部
/// \param[in] polygon 多边形顶点向量
/// \param[in] point 测试点坐标
/// \param[in] include_boundary 边界点是否被视为"内部"
/// \param[in] use_bbox_precheck 是否使用边界框预检查
/// \return 如果点在多边形内部则为true（边界行为未定义）
/// \return 如果点在外部或多边形顶点数<3则为false
/// \warning 没有显式边界处理 - 结果可能因浮点数精度而异
bool IsInSimplePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point,
                       bool include_boundary = true, bool use_bbox_precheck = true);

/// \brief 编译多边形，准备用于优化的射线投射算法
/// \details 预处理多边形，基于选定的算法:
///          - 对于kOptimizedRayCasting: 计算边数据、斜率和边界
///          - 对于kSimpleRayCasting: 只计算边界框
///          - 使用叉积测试检测退化（共线）多边形
///          - 如果存在则移除重复的结束顶点
/// \param[in] vertices 原始多边形顶点
/// \param[in] algorithm 要使用的算法
/// \return 编译后的多边形数据结构
CompiledPolygon CompilePolygon(const std::vector<Eigen::Vector2d> &vertices,
                               PointInPolygonAlgorithm algorithm = PointInPolygonAlgorithm::kOptimizedRayCasting);

/// \brief 移除多边形中的连续重复顶点
/// \param[in] vertices 输入多边形顶点
/// \param[in] epsilon 认为点为重复的容差
/// \return 移除重复后的新顶点向量
std::vector<Eigen::Vector2d> SanitizePolygonVertices(const std::vector<Eigen::Vector2d> &vertices,
                                                     double epsilon = 1e-9);

/// \brief 计算2D叉积
/// \param[in] a 第一个向量
/// \param[in] b 第二个向量
/// \return 两个向量的叉积
inline double Cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a.x() * b.y() - a.y() * b.x();
}

/// \brief 计算多边形面积的2倍（鞋带公式）以确定缠绕顺序
/// \param[in] vertices 输入多边形顶点
/// \return 多边形面积的2倍
double TwiceSignedArea(const std::vector<Eigen::Vector2d> &vertices);

Eigen::Vector2d LonLatToMercator(double lon_deg, double lat_deg);
std::vector<Eigen::Vector2d> ConvertPolygonLonLatToMercator(const std::vector<Eigen::Vector2d> &lonlat_points);

} // namespace geometry

#endif