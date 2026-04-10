// static_routines.cc
// Unidades: METROS (igual que el resto del framework).
// Las posiciones neutras se toman de context_.legs[i].idle_R,
// que ya contiene idle_x, idle_y y stand_height de tum.cfg.

#include "mech/static_routines.h"
#include "mech/quadruped_context.h"

#include <algorithm>
#include <cmath>

#include "base/common.h"

namespace mjmech {
namespace mech {

namespace {
inline bool isRight(int leg) { return (leg % 2) == 1; }
inline bool isFront(int leg) { return leg < 2; }
constexpr double deg2rad(double d) { return d * M_PI / 180.0; }
}  // namespace

StaticRoutines::StaticRoutines(const QuadrupedContext& context,
                               const QuadrupedConfig&  config)
    : context_(context), config_(config) {}

void StaticRoutines::Start(RoutineId id, RoutineState* state) const {
  *state = RoutineState{};
  state->routine        = id;
  state->step           = 0;
  state->step_elapsed_s  = 0.0;
  state->done           = false;
  const auto seq = BuildSequence(id);
  if (!seq.empty()) {
    state->step_duration_s = seq[0].duration_s;
  }
}

void StaticRoutines::Update(
    RoutineState*                              state,
    double                                     period_s,
    const std::vector<QuadrupedCommand::Leg>&  old_legs_R,
    std::vector<QuadrupedCommand::Leg>*        out_legs_R,
    base::KinematicRelation*                   out_RB) const {

  if (state->done) {
    *out_legs_R = old_legs_R;
    *out_RB     = context_.LevelDesiredRB();
    return;
  }

  const auto seq = BuildSequence(state->routine);

  if (state->step >= static_cast<int>(seq.size())) {
    // Si el último paso tiene hold_forever, retroceder al último paso y
    // mantenerlo activo indefinidamente. done nunca se pone en true.
    // El robot conserva la pose hasta que el usuario cambia de modo.
    const RoutineStep& last = seq.back();
    if (last.hold_forever) {
      state->step = static_cast<int>(seq.size()) - 1;
      state->step_elapsed_s  = 0.0;
      state->step_duration_s = last.duration_s;
      // Caer al código de Update normal para ese paso.
    } else {
      state->done = true;
      *out_legs_R = old_legs_R;
      *out_RB     = context_.LevelDesiredRB();
      return;
    }
  }

  const RoutineStep& target = seq[state->step];

  auto legs_R = old_legs_R;
  if (legs_R.empty()) {
    for (const auto& leg : context_.legs) {
      QuadrupedCommand::Leg lc;
      lc.leg_id    = leg.leg;
      lc.power     = true;
      lc.stance    = 1.0;
      lc.landing   = false;
      lc.kp_N_m    = config_.default_kp_N_m;
      lc.kd_N_m_s  = config_.default_kd_N_m_s;
      lc.position  = leg.idle_R;
      lc.velocity  = base::Point3D();
      legs_R.push_back(lc);
    }
  }

  for (auto& leg_R : legs_R) {
    leg_R.kp_N_m   = config_.default_kp_N_m;
    leg_R.kd_N_m_s = config_.default_kd_N_m_s;
    leg_R.kp_scale = {};
    leg_R.kd_scale = {};
    leg_R.stance   = 1.0;
    leg_R.landing  = false;
    leg_R.velocity.head<2>() = Eigen::Vector2d(0., 0.);
  }

  std::vector<std::pair<int, base::Point3D>> targets;
  for (const auto& leg : context_.legs) {
    targets.push_back({leg.leg, target.feet_R[leg.leg]});
  }

  const double remaining_s = std::max(
      state->step_duration_s - state->step_elapsed_s, period_s);

  double max_dist = 0.0;
  for (const auto& leg : context_.legs) {
    for (const auto& old : legs_R) {
      if (old.leg_id == leg.leg) {
        max_dist = std::max(max_dist,
            (old.position - target.feet_R[leg.leg]).norm());
        break;
      }
    }
  }
  const double safe_velocity = std::max(
      (remaining_s > 0.0) ? (max_dist / remaining_s) : 0.5,
      0.05);

  QuadrupedContext::MoveOptions move_opts;
  move_opts.override_acceleration = config_.stand_up.acceleration;

  const bool arrived = context_.MoveLegsFixedSpeed(
      &legs_R, safe_velocity, targets, move_opts);

  base::KinematicRelation desired_RB = context_.LevelDesiredRB();
  desired_RB.pose = target.body_offset_RB * desired_RB.pose;

  *out_legs_R = std::move(legs_R);
  *out_RB     = desired_RB;

  state->step_elapsed_s += period_s;
  if (state->step_elapsed_s >= state->step_duration_s &&
    state->step_elapsed_s > period_s){
    state->step++;
    state->step_elapsed_s = 0.0;
    if (state->step < static_cast<int>(seq.size())) {
      state->step_duration_s = seq[state->step].duration_s;
    } else {
      // Secuencia agotada. Si el último paso tiene hold_forever, quedarse
      // en él indefinidamente en lugar de marcar done.
      if (seq.back().hold_forever) {
        state->step         = static_cast<int>(seq.size()) - 1;
        state->step_duration_s = seq.back().duration_s;
        // step_elapsed_s ya es 0.0 — el paso se repite desde el inicio.
      } else {
        state->done = true;
      }
    }
  }
}

RoutineStep StaticRoutines::MakeStep(
    double x_front, double x_rear,
    double y_offset,
    double z_add_front, double z_add_rear,
    double duration_s,
    Sophus::SE3d body_offset) const {

  RoutineStep step;
  step.duration_s     = duration_s;
  step.body_offset_RB = body_offset;

  for (int i = 0; i < 4; i++) {
    base::Point3D pos = context_.legs[i].idle_R;
    pos.x() += isFront(i) ? x_front : x_rear;
    pos.y() += isRight(i) ? -y_offset : y_offset;
    pos.z() += isFront(i) ? z_add_front : z_add_rear;
    step.feet_R[i] = pos;
  }
  return step;
}

std::vector<RoutineStep> StaticRoutines::BuildSequence(RoutineId id) const {
  switch (id) {
    case RoutineId::kFlexion:    return BuildFlexion();
    case RoutineId::kBaile:      return BuildBaile();
    case RoutineId::kSentarse:   return BuildSentarse();
    case RoutineId::kLevantarse: return BuildLevantarse();
  }
  return {};
}

std::vector<RoutineStep> StaticRoutines::BuildFlexion() const {
  std::vector<RoutineStep> seq;
  for (int rep = 0; rep < 4; rep++) {
    Sophus::SE3d down;
    down.translation() = Eigen::Vector3d(0, 0, 0.07);
    seq.push_back(MakeStep(0, 0, 0, 0, 0, 1.0, down));
    seq.push_back(MakeStep(0, 0, 0, 0, 0, 1.0));
  }
  return seq;
}

std::vector<RoutineStep> StaticRoutines::BuildBaile() const {
  std::vector<RoutineStep> seq;
  auto bodyRoll = [&](double roll_deg, double pitch_deg,
                       double yaw_deg,  double dur) {
    Sophus::SE3d offset;
    offset.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(
            deg2rad(roll_deg),
            deg2rad(pitch_deg),
            deg2rad(yaw_deg)).eigen());
    return MakeStep(0, 0, 0, 0, 0, dur, offset);
  };
  for (int i = 0; i < 4; i++) {
    seq.push_back(bodyRoll(-10, 10,  5, 2.0));
    seq.push_back(bodyRoll( 10,  -10,  5, 2.0));
  }
  seq.push_back(MakeStep(0, 0, 0, 0, 0, 2.0));
  seq.push_back(MakeStep(-0.03, -0.03, 0, 0, 0, 2.0));
  seq.push_back(MakeStep(0, 0, 0, 0, 0, 2.0));
  for (int i = 0; i < 4; i++) {
    seq.push_back(bodyRoll(0, -10, 0, 2.0));
    seq.push_back(bodyRoll(0,  10, 0, 2.0));
  }
  seq.push_back(MakeStep(0, 0, 0, 0, 0, 2.0));
  return seq;
}

std::vector<RoutineStep> StaticRoutines::BuildSentarse() const {
  // Estrategia: bajar el CUERPO progresivamente con body_offset.z positivo.
  //
  // Razonamiento:
  //   stand_height = 0.305m → idle_R.z = 0.305m en frame R.
  //   El clamp en ControlLegs_B() limita leg_B.z <= max_z_B (0.300m por defecto).
  //   Como leg_B ≈ pose_BR * leg_R, si body_offset.z = +D (cuerpo baja D metros),
  //   entonces leg_B.z ≈ leg_R.z - D = 0.305 - D → pasa el clamp para D > 0.005m.
  //   Las patas permanecen fijas en frame R (z_add = 0); el efecto visual de
  //   sentarse viene del cuerpo acercándose al suelo, igual que en el código
  //   original (static_routines.cpp) que usaba body_p.z para la misma función.
  //
  // Verificación del clamp por paso (z_pie_B = 0.305 - D):
  //   Paso 1: D=0.040 → z_pie_B=0.265m ✓
  //   Paso 2: D=0.060 → z_pie_B=0.245m ✓
  //   Paso 3: D=0.090 → z_pie_B=0.215m ✓
  //   Paso 4: D=0.120 → z_pie_B=0.185m ✓
  //   Paso 5: D=0.150 → z_pie_B=0.155m ✓
  //   Paso 6: D=0.155 → z_pie_B=0.150m ✓
  //
  // Las patas traseras se mueven hacia atrás en X para que el robot
  // no caiga hacia atrás al bajar el centro de masa.

  std::vector<RoutineStep> seq;

  // Paso 1 — preparación: cuerpo baja levemente, patas neutras.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0, 0, 0.040);
    seq.push_back(MakeStep(0, 0, 0, 0, 0, 1.5, b));
  }

  // Paso 2 — patas traseras empiezan a retrasarse, cuerpo sigue bajando.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0, 0, 0.060);
    seq.push_back(MakeStep(0, -0.060, 0, 0, 0, 2.0, b));
  }

  // Paso 3 — cuerpo baja más, patas traseras más atrás.
  // body_offset.x empieza a desplazar el cuerpo hacia adelante (frame R).
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0.010, 0, 0.090);
    seq.push_back(MakeStep(0.020, -0.100, 0, 0, 0, 2.0, b));
  }

  // Paso 4 — postura sentado intermedia. Cuerpo más adelante, pitch para naturalidad.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0.025, 0, 0.120);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(-3.0), 0.0).eigen());
    seq.push_back(MakeStep(0.030, -0.110, 0, 0, 0, 2.0, b));
  }

  // Paso 5 — postura sentado profunda.
  // body_offset.x = 0.040m replica el body_p.x=+20mm del código viejo
  // con margen extra para vencer el filtro rb_filter_constant_Hz.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0.040, 0, 0.150);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(-3.0), 0.0).eigen());
    seq.push_back(MakeStep(0.040, -0.110, 0, 0, 0, 2.5, b));
  }

  // Paso 6 — postura final estable con hold_forever.
  // Mantiene esta pose indefinidamente hasta cambio de modo explícito.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0.040, 0, 0.155);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(-3.0), 0.0).eigen());
    RoutineStep s = MakeStep(0.040, -0.110, 0, 0, 0, 1.0, b);
    s.hold_forever = true;
    seq.push_back(s);
  }

  return seq;
}

std::vector<RoutineStep> StaticRoutines::BuildLevantarse() const {
  // Inverso de BuildSentarse: sube el cuerpo en tres pasos.
  // Asume que el robot arranca en la postura final de BuildSentarse
  // (body_offset.z ≈ 0.155m, x_rear=-0.110, x_front=+0.040).
  // Cada paso reduce D y devuelve las patas a la posición neutra.

  std::vector<RoutineStep> seq;

  // Paso 1 — desde sentado profundo, comenzar a subir.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0.020, 0, 0.155);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(-3.0), 0.0).eigen());
    seq.push_back(MakeStep(0.040, -0.110, 0, 0, 0, 0.5, b));  // breve hold
  }

  // Paso 2 — postura intermedia: cuerpo sube a D=0.080, patas vuelven parcialmente.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0.010, 0, 0.080);
    seq.push_back(MakeStep(0.020, -0.060, 0, 0, 0, 2.0, b));
  }

  // Paso 3 — volver a stand height con hold_forever: el controlador mantiene
  // esta pose (stand_height, patas neutras) sin transicionar a kRest.
  // La GUI puede lanzar kSentarse o cambiar de modo para salir.
  {
    RoutineStep s = MakeStep(0, 0, 0, 0, 0, 1.5);
    s.hold_forever = true;
    seq.push_back(s);
  }

  return seq;
}

}  // namespace mech
}  // namespace mjmech