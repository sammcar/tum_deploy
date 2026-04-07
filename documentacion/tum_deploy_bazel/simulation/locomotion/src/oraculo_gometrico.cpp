#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <cmath>
#include <vector>
#include <limits>

// Asumimos que tienes tu función IK disponible en tu código:
// LegAngles solve_IK(double x, double y, double z, bool es_pata_derecha);

namespace mjmech {
namespace mech {

// =================================================================
// 1. DEFINICIONES DE TIPOS PARA BOOST.GEOMETRY
// =================================================================
using Point = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point>;

// =================================================================
// 2. HERRAMIENTAS INTERNAS
// =================================================================
namespace { 

    double WrapNegPiToPi(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    double StraightLine(const Eigen::Vector2d& velocity, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
        const double x1 = 0, y1 = 0, x2 = velocity.x(), y2 = velocity.y();
        const double x3 = p1.x(), y3 = p1.y(), x4 = p2.x(), y4 = p2.y();
        const double denom = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
        if (denom == 0.0) return std::numeric_limits<double>::infinity();
        return ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    }

    double CurvedPath(const Eigen::Vector2d& velocity, double omega, const Eigen::Vector2d& p1_in, const Eigen::Vector2d& p2_in) {
        const double radius = velocity.norm() / omega;
        const Eigen::Vector2d center = (1.0 / omega) * ((Eigen::Matrix2d() << 0, -1, 1, 0).finished() * velocity);
        const Eigen::Vector2d p1 = p1_in - center;
        const Eigen::Vector2d p2 = p2_in - center;
        const double dx = p2.x() - p1.x(), dy = p2.y() - p1.y();
        const double dr2 = dx * dx + dy * dy;
        const double D = p1.x() * p2.y() - p2.x() * p1.y();
        const double discriminant2 = radius * radius * dr2 - D * D;
        if (discriminant2 < 0.0) return std::numeric_limits<double>::infinity();

        auto calct = [&](const Eigen::Vector2d& p) {
            Eigen::Vector2d delta = p - center;
            return WrapNegPiToPi(std::atan2(delta.y(), delta.x()) - std::atan2(-center.y(), -center.x())) * radius / velocity.norm();
        };
        const double discriminant = std::sqrt(discriminant2);
        auto sign = [](double v) { return v < 0.0 ? -1.0 : 1.0; };
        Eigen::Vector2d s1((D*dy + sign(dy)*dx*discriminant)/dr2 + center.x(), (-D*dx + std::abs(dy)*discriminant)/dr2 + center.y());
        Eigen::Vector2d s2((D*dy - sign(dy)*dx*discriminant)/dr2 + center.x(), (-D*dx - std::abs(dy)*discriminant)/dr2 + center.y());
        const double t1 = calct(s1), t2 = calct(s2);
        return (std::abs(t1) < std::abs(t2)) ? t1 : t2;
    }

        Polygon SearchPlane(const Eigen::Vector3d& center_p, bool es_derecha, const Polygon& exclude_region_G) {
        Polygon poly;
        const double kStep = 0.01; // Resolución del escaneo: 1 cm
        std::vector<Point> upper_points;
        std::vector<Point> lower_points;

        // 1. Encontrar los extremos en el eje X (Asume que el centro es la parte más ancha)
        double x_min = center_p.x();
        double x_max = center_p.x();
        while(solve_IK(x_max + kStep, center_p.y(), center_p.z(), es_derecha).valid) x_max += kStep;
        while(solve_IK(x_min - kStep, center_p.y(), center_p.z(), es_derecha).valid) x_min -= kStep;

        // Reservamos memoria aproximada para evitar re-alojamientos (Opcional pero recomendado)
        int estimated_points = (x_max - x_min) / kStep + 1;
        upper_points.reserve(estimated_points);
        lower_points.reserve(estimated_points);

        // 2. Escanear en Y para cada X formando los límites del polígono
        for (double x = x_min; x <= x_max; x += kStep) {
            double y_up = center_p.y();
            while(solve_IK(x, y_up + kStep, center_p.z(), es_derecha).valid) {
                // Si entra en la zona de exclusión (el cuerpo), nos detenemos
                if (!boost::geometry::disjoint(Point(x, y_up + kStep), exclude_region_G)) break;
                y_up += kStep;
            }
            
            double y_down = center_p.y();
            while(solve_IK(x, y_down - kStep, center_p.z(), es_derecha).valid) {
                if (!boost::geometry::disjoint(Point(x, y_down - kStep), exclude_region_G)) break;
                y_down -= kStep;
            }
            
            upper_points.push_back(Point(x, y_up));
            // OPTIMIZACIÓN: Lo guardamos normal con push_back, es mucho más rápido [4]
            lower_points.push_back(Point(x, y_down)); 
        }

        // 3. Unir todo en el anillo perimetral
        for(const auto& pt : upper_points) {
            boost::geometry::append(poly.outer(), pt);
        }
        
        // OPTIMIZACIÓN: Iteramos el vector inferior en reversa (rbegin a rend)
        for(auto it = lower_points.rbegin(); it != lower_points.rend(); ++it) {
            boost::geometry::append(poly.outer(), *it);
        }
        
        if (!upper_points.empty()) {
            boost::geometry::append(poly.outer(), upper_points.front());
        }
        
        boost::geometry::correct(poly); // Asegura orientación horaria estándar [1]
        return poly;
    }

    
} // namespace anónimo

double TrajectoryLineIntersectTime(const Eigen::Vector2d& velocity, double omega, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    if (std::abs(omega) < 1e-6) return StraightLine(velocity, p1, p2);
    else return CurvedPath(velocity, omega, p1, p2);
}

// =================================================================
// 3. CLASE PRINCIPAL: ValidLegRegion
// =================================================================
class ValidLegRegion {
public:
    // NUEVO CONSTRUCTOR: Dibuja dinámicamente el polígono basado en tu cinemática
    ValidLegRegion(const Eigen::Vector3d& idle_G, double lift_height, bool es_derecha) {
        std::vector<Polygon> poly_G;
        
        // Define una zona de exclusión básica (por ejemplo, evitar chocar con la panza)
        Polygon exclude_region_G; 
        boost::geometry::read_wkt("POLYGON((-0.1 -0.05, -0.1 0.05, 0.1 0.05, 0.1 -0.05, -0.1 -0.05))", exclude_region_G);

        // Escaneamos el espacio en el suelo (Z) y en el punto más alto del paso (Z + Lift)
        for (double z_offset : { 0.0, lift_height }) {
            Eigen::Vector3d p_G = idle_G;
            // Sumamos z_offset asumiendo que tu suelo está en negativo (ej -0.306) y el levantamiento sube hacia 0.
            p_G.z() = idle_G.z() + z_offset; 
            
            poly_G.push_back(SearchPlane(p_G, es_derecha, exclude_region_G));
        }

        std::vector<Polygon> merged_G;
        // Solo la región que es válida tanto en el aire como en el suelo es 100% segura
        boost::geometry::intersection(poly_G.front(), poly_G.back(), merged_G);
        
        if (merged_G.empty()) {
            std::cerr << "[WARNING] IK no encontró región válida. Usando polígono de seguridad estático.\n";
            boost::geometry::read_wkt("POLYGON((-0.1 -0.1, -0.1 0.1, 0.1 0.1, 0.1 -0.1, -0.1 -0.1))", bounds_G_);
            return;
        }

        // Simplificamos el polígono para que los cálculos de intersección en tiempo real sean ultra rápidos
        double kXStep = 0.01; 
        boost::geometry::simplify(merged_G.front(), bounds_G_, kXStep);
    }

    double TimeToLeave_G(const Eigen::Vector2d& point_G, const Eigen::Vector2d& velocity, double omega) const {
        if (!boost::geometry::within(Point(point_G.x(), point_G.y()), bounds_G_)) return -1.0;

        double min_time = std::numeric_limits<double>::infinity();
        const auto& ring = bounds_G_.outer();
        for (size_t i = 0; i < ring.size() - 1; ++i) {
            Eigen::Vector2d p1(ring[i].x() - point_G.x(), ring[i].y() - point_G.y());
            Eigen::Vector2d p2(ring[i+1].x() - point_G.x(), ring[i+1].y() - point_G.y());
            double t = TrajectoryLineIntersectTime(velocity, omega, p1, p2);
            if (t >= 0.0 && t < min_time) min_time = t;
        }
        return min_time;
    }

private:
    Polygon bounds_G_;
};

} // namespace mech
} // namespace mjmech