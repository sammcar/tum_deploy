#pragma once

// static_routines.h
//
// Módulo de rutinas estáticas para ATOM-51.
// StaticRoutines es llamado por DoControl_Routine() cada ciclo de control.
// No depende de SHM ni de motores — solo habla legs_R en frame R.
//
// NOTA DE DEPENDENCIAS:
//   Este header NO incluye quadruped_context.h para evitar la dependencia
//   circular:
//     quadruped_context.h → quadruped_state.h → static_routines.h
//                                             → quadruped_context.h  ← CICLO
//   Se usa una forward-declaration de QuadrupedContext; la definición completa
//   se incluye solo en static_routines.cc.

#include <string>
#include <vector>

#include "sophus/se3.hpp"
#include "mjlib/base/visitor.h"

#include "base/point3d.h"
#include "base/kinematic_relation.h"
#include "base/quaternion.h"
#include "mech/quadruped_command.h"
#include "mech/quadruped_config.h"

namespace mjmech {
namespace mech {

// ---------------------------------------------------------------------------
// Identificadores de rutina.
// ---------------------------------------------------------------------------
enum class RoutineId {
  kFlexion,
  kBaile,
  kSentarse,
  kLevantarse,
  kSaludo,
};

// ---------------------------------------------------------------------------
// RoutineState — almacenado en QuadrupedState.
// Solo primitivos; no depende de QuadrupedContext.
// ---------------------------------------------------------------------------
struct RoutineState {
  RoutineId routine        = RoutineId::kFlexion;
  int       step           = 0;
  double    step_elapsed_s  = 0.0;
  double    step_duration_s = 0.0;
  bool      done           = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(step));
    a->Visit(MJ_NVP(step_elapsed_s));
    a->Visit(MJ_NVP(step_duration_s));
    a->Visit(MJ_NVP(done));
  }
};

// ---------------------------------------------------------------------------
// RoutineStep — un fotograma de la coreografía.
// ---------------------------------------------------------------------------
struct RoutineStep {
  base::Point3D feet_R[4];       // posición objetivo de cada pie en frame R
  Sophus::SE3d  body_offset_RB;  // composición sobre LevelDesiredRB().pose
  double        duration_s = 1.0;
  // Si true, este paso se repite indefinidamente cuando es el último de la
  // secuencia: done nunca se pone en true y el robot mantiene la pose hasta
  // que el usuario cambia de modo. Usado por kSentarse y kLevantarse.
  bool          hold_forever = false;
};

// ---------------------------------------------------------------------------
// Forward-declaration — evita incluir quadruped_context.h desde aquí.
// La definición completa se incluye en static_routines.cc.
// ---------------------------------------------------------------------------
struct QuadrupedContext;

// ---------------------------------------------------------------------------
// StaticRoutines — clase principal del módulo.
// ---------------------------------------------------------------------------
class StaticRoutines {
 public:
  explicit StaticRoutines(const QuadrupedContext& context,
                          const QuadrupedConfig&  config);

  // Reinicia la máquina de estados para la rutina indicada.
  void Start(RoutineId id, RoutineState* state) const;

  // Llamado una vez por ciclo de control desde DoControl_Routine().
  //
  //   out_RB: tipo base::KinematicRelation, el mismo que devuelve
  //           QuadrupedContext::LevelDesiredRB().
  void Update(RoutineState*                              state,
              double                                     period_s,
              const std::vector<QuadrupedCommand::Leg>&  old_legs_R,
              std::vector<QuadrupedCommand::Leg>*        out_legs_R,
              base::KinematicRelation*                   out_RB) const;

 private:
  std::vector<RoutineStep> BuildSequence(RoutineId id)  const;
  std::vector<RoutineStep> BuildFlexion()               const;
  std::vector<RoutineStep> BuildBaile()                 const;
  std::vector<RoutineStep> BuildSentarse()              const;
  std::vector<RoutineStep> BuildLevantarse()            const;
  std::vector<RoutineStep> BuildSaludo()                const;

  RoutineStep MakeStep(double x_front,    double x_rear,
                       double y_offset,
                       double z_add_front, double z_add_rear,
                       double duration_s,
                       Sophus::SE3d body_offset = Sophus::SE3d()) const;

  const QuadrupedContext& context_;
  const QuadrupedConfig&  config_;
};

}  // namespace mech
}  // namespace mjmech