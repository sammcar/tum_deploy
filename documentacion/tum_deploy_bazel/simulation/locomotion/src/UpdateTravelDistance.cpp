// Se asume el uso de Sophus::SE3d para las transformaciones, que es lo que usa el original.
#include <sophus/se3.hpp>

struct LegData {
    Eigen::Vector3d idle_R;        // Posición ideal de descanso (marco R)
    ValidLegRegion* region;        
    double travel_distance = 0.0;  
    
    // NUEVO: La transformación estática de dónde está anclada la pata en el robot
    Sophus::SE3d pose_BG;          
};

void UpdateTravelDistance(
    std::vector<LegData>& legs, 
    const Eigen::Vector3d& desired_v,  
    const Eigen::Vector3d& desired_w,
    const Sophus::SE3d& frame_RB)  // NUEVO: La pose dinámica actual del cuerpo del robot
{
    for (int id = 0; id < 4; id++) {
        
        // --- LA TRANSFORMACIÓN DE JOSH ---
        // 1. Posición base ideal (R)
        const Eigen::Vector3d p_R = legs[id].idle_R; 
        
        // 2. Transformar al marco del Cuerpo (B).
        // Al multiplicar por la inversa de frame_RB, estamos metiendo en la ecuación 
        // la inclinación, balanceo y altura actuales del robot.
        const Eigen::Vector3d p_B = frame_RB.inverse() * p_R; 
        
        // 3. Transformar al marco local de la Pata (G).
        const Eigen::Vector3d p_G = legs[id].pose_BG.inverse() * p_B; 

        // Lambda para predecir el impacto evaluando una dirección (sign)
        auto find_distance = [&](double sign) {
            
            Eigen::Vector3d maybe_v = (sign * desired_v) + p_R.cross(desired_w);
            Eigen::Vector3d v = (maybe_v.norm() == 0.0) ? Eigen::Vector3d(0.1, 0.0, 0.0) : maybe_v;
            
            // 4. Usamos el p_G dinámico calculado, tomando solo (X, Y)
            double time_to_leave = legs[id].region->TimeToLeave_G(
                p_G.head<2>(), 
                v.head<2>(), 
                -desired_w.z()
            );

            constexpr double kBothSides = 2.0;
            return kBothSides * std::abs(v.norm() * time_to_leave);
        };
        
        legs[id].travel_distance = std::min(find_distance(-1.0), find_distance(1.0));
    }
}