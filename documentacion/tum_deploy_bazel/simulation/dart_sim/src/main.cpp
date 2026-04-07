#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// Asegúrate de que DART encuentre este archivo (quizás debas añadirlo al include de CMake)
#include "robot_types.hpp" 

using namespace dart::dynamics;
using namespace dart::simulation;

// ============================================================================
// EL NODO DE DART (Gemelo Digital + Firmware Moteus + Oráculo Físico)
// ============================================================================

class RobotController : public dart::gui::osg::WorldNode {
public:
    RobotController(WorldPtr world, SkeletonPtr robot, 
                    CommandData* cmd, TelemetryData* tel, IMUData* imu) 
        : WorldNode(world), mRobot(robot), mCmd(cmd), mTel(tel), mIMU(imu) {
        
        int idx = 0;
        for (int i = 0; i < 4; i++) {
            std::string sufijo = obtener_sufijo_pata(i);
            mJoints[idx++] = mRobot->getJoint("Coxa" + sufijo);
            mJoints[idx++] = mRobot->getJoint("Hip" + sufijo);
            mJoints[idx++] = mRobot->getJoint("Knee_" + sufijo);
        }
    }

    void customPreStep() override {
        // ==========================================================
        // 1. FASE DE LECTURA (DART -> Memoria Compartida de Locomotion)
        // ==========================================================

        static const bool invert_sign[4][3] = {
            // Coxa    Hip     Knee
            { false,  false,  false },  // FL (p=0)
            { true,   false,  false },  // FR (p=1)
            { false,  false,  false },  // BL (p=2)
            { true,   false,  false },  // BR (p=3)
        };

        
        // A. Escribir Telemetría de Motores
        for (int p = 0; p < 4; p++) {
            for (int m = 0; m < 3; m++) {
                int i = p * 3 + m;
                if (mJoints[i]) {
                    // DART trabaja en radianes, Locomotion espera GRADOS
                    double pos_deg = mJoints[i]->getPosition(0) * 180.0 / M_PI;
                    double vel_deg = mJoints[i]->getVelocity(0) * 180.0 / M_PI;

                    if (invert_sign[p][m]) {
                        pos_deg = -pos_deg;
                        vel_deg = -vel_deg;
                    }
                    
                    mTel->measured_angles[p][m] = pos_deg;
                    mTel->measured_velocities[p][m] = vel_deg;
                    mTel->measured_torques[p][m] = mJoints[i]->getForce(0);
                }
            }
        }

        // B. Escribir IMU Falsa
        dart::dynamics::BodyNode* rootBody = mRobot->getRootBodyNode();
        if (rootBody) {
            Eigen::Matrix3d rot = rootBody->getTransform().linear();
            Eigen::Vector3d euler_rad = dart::math::matrixToEulerXYZ(rot);
            mIMU->roll  = euler_rad.x() * 180.0 / M_PI;
            mIMU->pitch = euler_rad.y() * 180.0 / M_PI;
            mIMU->yaw   = euler_rad.z() * 180.0 / M_PI;

            Eigen::Vector3d omega_rad = rootBody->getSpatialVelocity().head<3>();
            mIMU->gyro[0] = omega_rad.x() * 180.0 / M_PI;
            mIMU->gyro[1] = omega_rad.y() * 180.0 / M_PI;
            mIMU->gyro[2] = omega_rad.z() * 180.0 / M_PI;

            Eigen::Vector3d acc_ms2 = rootBody->getLinearAcceleration();
            mIMU->acc[0] = acc_ms2.x() / 9.81;
            mIMU->acc[1] = acc_ms2.y() / 9.81;
            mIMU->acc[2] = (acc_ms2.z() / 9.81) + 1.0; 
        }

        // ==========================================================
        // 2. FASE DE FÍSICA (Calculadora de Gravedad)
        // ==========================================================
        Eigen::VectorXd CG_full = mRobot->getCoriolisAndGravityForces();
        Eigen::VectorXd CG_actuated = CG_full.segment<12>(6);

        // ==========================================================
        // 3. FASE DE ACTUACIÓN (Memoria Compartida -> DART)
        // ==========================================================
        double MAX_TORQUE = 15.0; 
        double BASE_KP = 320.0;
        double BASE_KD = 15.0;

        for (int p = 0; p < 4; p++) {
            for (int m = 0; m < 3; m++) {
                int i = p * 3 + m;
                if (mJoints[i]) {
                    // Locomotion escribe en GRADOS, convertimos a radianes para DART
                    double raw_deg = mCmd->angles[p][m];
                    double q_des = (invert_sign[p][m] ? -raw_deg : raw_deg) * M_PI / 180.0;
                    double dq_des = mCmd->velocities[p][m] * M_PI / 180.0;
                    
                    double current_q = mJoints[i]->getPosition(0);
                    double current_dq = mJoints[i]->getVelocity(0);
                    
                    // Locomotion envía una "escala" (0.0 a 1.0). Lo multiplicamos por la rigidez real.
                    double kp = BASE_KP * mCmd->kp_scale[p][m];
                    double kd = BASE_KD * mCmd->kd_scale[p][m];
                    
                    double error_q = q_des - current_q;
                    double error_dq = dq_des - current_dq;
                    
                    // El Feedforward es la gravedad calculada por DART
                    double tau_ff = CG_actuated(i); 

                    // Ecuación PD
                    double tau = (kp * error_q) + (kd * error_dq) + tau_ff;
                    
                    if (std::isnan(tau)) tau = 0.0; 
                    tau = std::clamp(tau, -MAX_TORQUE, MAX_TORQUE);
                    
                    mJoints[i]->setForce(0, tau);
                }
            }
        }
    }

private:
    SkeletonPtr mRobot;
    Joint* mJoints[12];
    CommandData* mCmd;
    TelemetryData* mTel;
    IMUData* mIMU;

    std::string obtener_sufijo_pata(int i) {
        if (i == 0) return "FL";
        if (i == 1) return "FR";
        if (i == 2) return "BL";
        return "BR";    
    }
};

// ============================================================================
// MAIN Y CONFIGURACIÓN FÍSICA
// ============================================================================

int main() {
    // --- 1. CONECTAR A LA MEMORIA COMPARTIDA ---
    int cmd_fd = shm_open("/rex_cmd", O_CREAT | O_RDWR, 0666);
    ftruncate(cmd_fd, sizeof(CommandData));
    CommandData* shm_ptr = static_cast<CommandData*>(mmap(0, sizeof(CommandData), PROT_READ | PROT_WRITE, MAP_SHARED, cmd_fd, 0));

    int tel_fd = shm_open("/rex_tel", O_CREAT | O_RDWR, 0666);
    ftruncate(tel_fd, sizeof(TelemetryData));
    TelemetryData* tel_ptr = static_cast<TelemetryData*>(mmap(0, sizeof(TelemetryData), PROT_READ | PROT_WRITE, MAP_SHARED, tel_fd, 0));

    int imu_fd = shm_open("/imu_data", O_CREAT | O_RDWR, 0666);
    ftruncate(imu_fd, sizeof(IMUData));
    IMUData* imu_ptr = static_cast<IMUData*>(mmap(0, sizeof(IMUData), PROT_READ | PROT_WRITE, MAP_SHARED, imu_fd, 0));

    // --- 2. INICIAR DART ---
    WorldPtr world = World::create();
    world->setGravity(Eigen::Vector3d(0, 0, -9.81));
    world->setTimeStep(0.001); 

    dart::utils::DartLoader loader;
    std::string absolute_urdf_dir = std::filesystem::absolute("../urdf").string();
    std::string absolute_urdf_file = std::filesystem::absolute("../urdf/tum.urdf").string();
    loader.addPackageDirectory("stl", absolute_urdf_dir + "/stl");

    SkeletonPtr robot = loader.parseSkeleton(absolute_urdf_file);
    if (!robot) return -1;
    
    dart::dynamics::Joint* root_joint = robot->getRootBodyNode()->getParentJoint();
    Eigen::Isometry3d tf_robot = Eigen::Isometry3d::Identity();
    tf_robot.translation() = Eigen::Vector3d(0.0, 0.0, 0.40);
    root_joint->setTransformFromParentBodyNode(tf_robot);

    for (size_t i = 0; i < robot->getNumJoints(); ++i) {
        auto joint = robot->getJoint(i);
        if (joint->getNumDofs() == 1) { 
            joint->setActuatorType(dart::dynamics::Joint::FORCE);
            joint->setSpringStiffness(0, 0.0);   
            joint->setDampingCoefficient(0, 0.1); 
        }
    }
    world->addSkeleton(robot);

    SkeletonPtr ground = Skeleton::create("ground");
    BodyNodePtr groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
    ShapePtr groundBox = std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1));
    auto shapeNode = groundBody->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(groundBox);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.5, 0.5, 0.5)); 
    
    Eigen::Isometry3d tf_ground = Eigen::Isometry3d::Identity();
    tf_ground.translation() = Eigen::Vector3d(0.0, 0.0, -0.25); 
    groundBody->getParentJoint()->setTransformFromParentBodyNode(tf_ground);
    shapeNode->getDynamicsAspect()->setFrictionCoeff(1.5);
    world->addSkeleton(ground);

    osg::ref_ptr<RobotController> node = new RobotController(world, robot, shm_ptr, tel_ptr, imu_ptr);
    node->setNumStepsPerCycle(15);

    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.setUpViewInWindow(0, 0, 1280, 720);
    viewer.getCameraManipulator()->setHomePosition(osg::Vec3(1.5, 1.5, 1.5), osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 1));
    viewer.home();

    std::cout << "Simulador listo. Esperando comandos de locomotion por Memoria Compartida..." << std::endl;
    viewer.simulate(true); 
    viewer.run();

    return 0;
}