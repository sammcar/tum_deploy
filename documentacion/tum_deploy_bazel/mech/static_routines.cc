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
    case RoutineId::kSaludo:     return BuildSaludo();
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

// RUTINA DE ACOSTARSE — pendiente de tuning. Conservada para uso futuro.
// std::vector<RoutineStep> StaticRoutines::BuildSentarse() const {
//   // Meta: acostar el robot con la panza apoyada en el suelo y patas en el aire.
//   //
//   // Dos fases:
//   //   Fase 1 (pasos 1-6): replica el código original — patas se doblan y retroceden.
//   //   Fase 2 (pasos 7-12): descenso del cuerpo combinando body_offset.z (baja)
//   //     y body_offset.x creciente (adelante). El cuerpo se desplaza sobre las patas,
//   //     que quedan libres hacia atrás sin necesidad de rotación.
//   //
//   // Regla de seguridad: z_pie_B = (0.305 + z_add) - D >= 0.005m
//   //
//   // Verificación de clamps (z_pie_B front / rear):
//   //   Paso 1:  0.295 / 0.235 ✓
//   //   Paso 2:  0.295 / 0.235 ✓
//   //   Paso 3:  0.245 / 0.225 ✓
//   //   Paso 4:  0.145 / 0.115 ✓
//   //   Paso 5:  0.105 / 0.075 ✓
//   //   Paso 6:  0.095 / 0.065 ✓
//   //   Paso 7:  0.070 / 0.040 ✓  x=+0.060
//   //   Paso 8:  0.010 / 0.010 ✓  x=+0.090
//   //   Paso 9:  0.010 / 0.010 ✓  x=+0.110
//   //   Paso 10: 0.010 / 0.010 ✓  x=+0.120
//   //   Paso 11: 0.010 / 0.010 ✓  x=+0.100
//   //   Paso 12: 0.015 / 0.015 ✓  x=+0.060  hold_forever
// 
//   std::vector<RoutineStep> seq;
// 
//   // --- FASE 1: doblar patas y preparar equilibrio ---
// 
//   // Paso 1 — traseras empiezan a doblarse. D=0.010 para pasar max_z_B.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0, 0, 0.010);
//     seq.push_back(MakeStep(0, 0, 0, 0, -0.060, 1.5, b));
//   }
// 
//   // Paso 2 — todas retroceden, traseras siguen dobladas.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0, 0, 0.010);
//     seq.push_back(MakeStep(-0.050, -0.050, 0, 0, -0.060, 2.0, b));
//   }
// 
//   // Paso 3 — retroceso mayor, delanteras empiezan a doblarse.
//   seq.push_back(MakeStep(-0.080, -0.080, 0, -0.060, -0.080, 2.0));
// 
//   // Paso 4 — patas muy dobladas, pitch leve.
//   {
//     Sophus::SE3d b;
//     b.so3() = Sophus::SO3d(
//         base::Quaternion::FromEuler(0.0, deg2rad(-3.0), 0.0).eigen());
//     seq.push_back(MakeStep(-0.110, -0.110, 0, -0.160, -0.190, 2.0, b));
//   }
// 
//   // Paso 5 — cuerpo empieza a bajar y avanza.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.020, 0, 0.040);
//     b.so3() = Sophus::SO3d(
//         base::Quaternion::FromEuler(0.0, deg2rad(-3.0), 0.0).eigen());
//     seq.push_back(MakeStep(-0.110, -0.110, 0, -0.160, -0.190, 2.5, b));
//   }
// 
//   // Paso 6 — apertura lateral inicial, patas aún más dobladas.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.020, 0, 0.040);
//     seq.push_back(MakeStep(-0.110, -0.110, 0.020, -0.170, -0.200, 1.5, b));
//   }
// 
//   // --- FASE 2: descenso con traslación X creciente ---
//   // El cuerpo avanza sobre las patas (X+) mientras baja (Z+).
//   // Las patas traseras quedan libres hacia atrás sin necesidad de rotación.
// 
//   // Paso 7 — cuerpo avanza y baja un poco. D=0.070, x=+0.060.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.060, 0, 0.070);
//     seq.push_back(MakeStep(-0.110, -0.120, 0.030, -0.165, -0.195, 2.0, b));
//   }
// 
//   // Paso 8 — cuerpo avanza más. D=0.110, x=+0.090.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.090, 0, 0.110);
//     seq.push_back(MakeStep(-0.100, -0.130, 0.040, -0.185, -0.185, 2.0, b));
//   }
// 
//   // Paso 9 — descenso fuerte. D=0.160, x=+0.110.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.110, 0, 0.160);
//     seq.push_back(MakeStep(-0.090, -0.140, 0.050, -0.135, -0.135, 2.0, b));
//   }
// 
//   // Paso 10 — casi en el suelo. D=0.210, x=+0.120.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.120, 0, 0.210);
//     seq.push_back(MakeStep(-0.080, -0.150, 0.060, -0.085, -0.085, 2.0, b));
//   }
// 
//   // Paso 11 — cuerpo casi plano, x empieza a reducirse. D=0.250, x=+0.100.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.100, 0, 0.250);
//     seq.push_back(MakeStep(-0.070, -0.160, 0.065, -0.045, -0.045, 2.0, b));
//   }
// 
//   // Paso 12 — postura final acostada. D=0.270, x=+0.060. hold_forever.
//   // Panza apoyada en el suelo, coxas abiertas, patas en el aire.
//   {
//     Sophus::SE3d b;
//     b.translation() = Eigen::Vector3d(0.060, 0, 0.270);
//     RoutineStep s = MakeStep(-0.060, -0.170, 0.070, -0.020, -0.020, 1.0, b);
//     s.hold_forever = true;
//     seq.push_back(s);
//   }
// 
//   return seq;
// }
// 

std::vector<RoutineStep> StaticRoutines::BuildSentarse() const {
  // Sentado de perro: parte trasera baja al suelo, delanteras rectas.
  //
  // Mecanismo: body_offset combina pitch positivo (nariz sube, cola baja)
  // con traslación Z positiva (cuerpo baja globalmente). El pitch hace que
  // la parte trasera del cuerpo se acerque al suelo mientras las patas
  // delanteras permanecen extendidas soportando el frente.
  // Las patas traseras se doblan naturalmente al bajar las caderas.
  // Las patas delanteras se mueven levemente hacia adelante para equilibrio.
  //
  // Postura final: hold_forever hasta que el usuario lance kLevantarse.

  std::vector<RoutineStep> seq;

  // Paso único — pitch +12°, D=0.015. hold_forever.
  {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0, 0, 0.015);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(12.0), 0.0).eigen());
    RoutineStep s = MakeStep(0, 0, 0, 0, 0, 2.0, b);
    s.hold_forever = true;
    seq.push_back(s);
  }

  return seq;
}

// LEVANTARSE (par de acostarse) — pendiente de tuning.
// (código comentado arriba)

std::vector<RoutineStep> StaticRoutines::BuildLevantarse() const {
  // Inverso exacto de BuildSentarse.
  // Desde hold de sentarse (pitch=+12°, D=0.025, x_front=+0.010):
  // Paso 1 replica el paso 1 de sentarse, luego vuelve a stand. hold_forever.

  std::vector<RoutineStep> seq;

  // Paso único — inverso exacto: pitch 0°, D=0. hold_forever.
  {
    RoutineStep s = MakeStep(0, 0, 0, 0, 0, 2.0);
    s.hold_forever = true;
    seq.push_back(s);
  }

  return seq;
}

std::vector<RoutineStep> StaticRoutines::BuildSaludo() const {
  // Rutina de saludo con pata FR (leg_id=1, tibia=servo 6).
  // Debe lanzarse desde el hold de kSentarse.
  //
  // El cuerpo y las 3 patas normales se gestionan aquí (frame R + body_offset).
  // El motor 6 se gestiona en quadruped_control.cc leyendo routine.step.
  //
  // Mapeo de pasos:
  //   Paso 0 (1.5s): Cuerpo traslada Y=+0.04 (alejarse de FR). Pitch +7°.
  //   Paso 1 (1.2s): Cuerpo llegó. Motor 6 sube +25° (override en .cc).
  //   Pasos 2-6    : Wave del motor 6: +5 -10 +10 -10 +5 (0.4s cada uno).
  //   Paso 7 (1.2s): Motor 6 baja de vuelta a posición sentarse.
  //   Paso 8 (1.5s): Cuerpo vuelve a Y=0. Pitch +7°. hold_forever.

  std::vector<RoutineStep> seq;

  // Pose base de sentarse — la misma que BuildSentarse() para consistencia.
  auto sentarse_body = [&]() {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(0, 0, 0.015);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(12.0), 0.0).eigen());
    return b;
  };

  // Pose con cuerpo desplazado en Y (lejos de pata FR = lado derecho).
  // Y positivo en body_offset.translation → cuerpo va a la izquierda
  // → peso se aleja de FR, FR puede levantar con seguridad.
  auto saludo_body = [&]() {
    Sophus::SE3d b;
    b.translation() = Eigen::Vector3d(-0.03, -0.06, 0.005);
    b.so3() = Sophus::SO3d(
        base::Quaternion::FromEuler(0.0, deg2rad(12.0), 0.0).eigen());
    return b;
  };

  // Paso 0 — traslación lateral del cuerpo. Motor 6 aún en reposo.
  seq.push_back(MakeStep(0, 0, 0, 0, 0, 1.5, saludo_body()));

  // Paso 1 — cuerpo ya desplazado, motor 6 empieza a subir (override .cc).
  seq.push_back(MakeStep(0, 0, 0, 0, 0, 1.2, saludo_body()));

  // Pasos 2-6 — wave. Cuerpo quieto. Motor 6 oscila (override .cc).
  // duración corta para movimiento ágil.
  for (int i = 0; i < 5; i++) {
    seq.push_back(MakeStep(0, 0, 0, 0, 0, 0.4, saludo_body()));
  }

  // Paso 7 — motor 6 baja de vuelta a ángulo de sentarse (override .cc).
  seq.push_back(MakeStep(0, 0, 0, 0, 0, 1.2, saludo_body()));

  // Paso 8 — cuerpo vuelve a Y=0 con pitch de sentarse. hold_forever.
  {
    RoutineStep s = MakeStep(0, 0, 0, 0, 0, 1.5, sentarse_body());
    s.hold_forever = true;
    seq.push_back(s);
  }

  return seq;
}

}  // namespace mech
}  // namespace mjmech