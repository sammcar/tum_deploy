#pragma once

#include <optional>

#include "sophus/se3.hpp"

#include "mjlib/base/visitor.h"

#include "base/kinematic_relation.h"
#include "base/point3d.h"
#include "base/quaternion.h"

#include "mech/quadruped_command.h"
// static_routines.h no incluye quadruped_context.h (usa forward-declaration),
// así que este include no introduce ningún ciclo.
#include "mech/static_routines.h"

namespace mjmech {
namespace mech {

struct QuadrupedState {
  // The joint level.
  struct Joint {
    int id = 0;

    double angle_deg = 0.0;
    double velocity_dps = 0.0;
    double torque_Nm = 0.0;

    double temperature_C = 0.0;
    double voltage = 0.0;
    int32_t mode = 0;
    int32_t fault = 0;

    double kp_Nm = 0.0;
    double ki_Nm = 0.0;
    double kd_Nm = 0.0;
    double feedforward_Nm = 0.0;
    double command_Nm = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(temperature_C));
      a->Visit(MJ_NVP(voltage));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(kp_Nm));
      a->Visit(MJ_NVP(ki_Nm));
      a->Visit(MJ_NVP(kd_Nm));
      a->Visit(MJ_NVP(feedforward_Nm));
      a->Visit(MJ_NVP(command_Nm));
    }
  };

  std::vector<Joint> joints;

  struct Leg {
    int leg = 0;
    base::Point3D position;
    base::Point3D velocity;
    base::Point3D force_N;
    double stance = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg));
      a->Visit(MJ_NVP(position));
      a->Visit(MJ_NVP(velocity));
      a->Visit(MJ_NVP(force_N));
      a->Visit(MJ_NVP(stance));
    }

    friend Leg operator*(const Sophus::SE3d&, const Leg& rhs);
  };

  std::vector<Leg> legs_B;

  struct Robot {
    base::KinematicRelation desired_R;
    base::KinematicRelation frame_RB;
    base::KinematicRelation frame_MB;
    base::KinematicRelation frame_AB;

    std::array<double, 2> terrain_rad = {};
    Sophus::SE3d tf_TA;

    double voltage = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(desired_R));
      a->Visit(MJ_NVP(frame_RB));
      a->Visit(MJ_NVP(frame_MB));
      a->Visit(MJ_NVP(frame_AB));
      a->Visit(MJ_NVP(terrain_rad));
      a->Visit(MJ_NVP(tf_TA));
      a->Visit(MJ_NVP(voltage));
    }
  };

  Robot robot;

  struct StandUp {
    enum Mode {
      kPrepositioning,
      kStanding,
      kDone,
    };

    Mode mode = kPrepositioning;
    int prepositioning_stage = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(prepositioning_stage));
    }
  };

  StandUp stand_up;

  struct Rest {
    bool done = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(done));
    }
  };

  Rest rest;

  struct Situp {
    enum Mode {
      kLowering = 0,
      kPulling  = 1,
      kDone     = 2,
    };

    Mode mode = kLowering;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
    }
  };

  Situp situp;

  struct Jump {
    enum Mode {
      kLowering   = 0,
      kPushing    = 1,
      kRetracting = 2,
      kFalling    = 3,
      kLanding    = 4,
      kDone       = 5,
    };

    Mode   mode         = kLowering;
    double velocity     = 0.0;
    double acceleration = 0.0;

    QuadrupedCommand::Jump command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(velocity));
      a->Visit(MJ_NVP(acceleration));
      a->Visit(MJ_NVP(command));
    }
  };

  Jump jump;

  struct Walk {
    int idle_count = 0;

    struct Leg {
      base::Point3D target_R;
      double invalid_time_s   = 0.0;
      double travel_distance  = 0.0;
      double restore_velocity = 0.0;
      double restore_remaining = 0.0;
      bool   latched          = false;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(target_R));
        a->Visit(MJ_NVP(invalid_time_s));
        a->Visit(MJ_NVP(travel_distance));
        a->Visit(MJ_NVP(restore_velocity));
        a->Visit(MJ_NVP(restore_remaining));
        a->Visit(MJ_NVP(latched));
      }
    };

    std::array<Leg, 4> legs;

    struct VLeg {
      double remaining_s      = 0.0;
      double invalid_time_s   = 0.0;
      enum Mode { kStance, kSwing, };
      Mode   mode             = kStance;
      double swing_elapsed_s  = 0.0;
      double stance_elapsed_s = 0.0;
      double phase_s          = 0.0;
      double target_time_s    = 0.0;
      Eigen::Vector3d predicted_next_v;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(remaining_s));
        a->Visit(MJ_NVP(invalid_time_s));
        a->Visit(MJ_NVP(mode));
        a->Visit(MJ_NVP(swing_elapsed_s));
        a->Visit(MJ_NVP(stance_elapsed_s));
        a->Visit(MJ_NVP(phase_s));
        a->Visit(MJ_NVP(target_time_s));
        a->Visit(MJ_NVP(predicted_next_v));
      }
    };

    std::array<VLeg, 2> vlegs;
    int           next_step_vleg  = 0;
    double        stance_elapsed_s = 0.0;
    base::Point3D last_swing_v_R;
    double        travel_distance  = 0.0;

    struct Trot {
      double swing_time   = 0.0;
      double onevleg_time = 0.0;
      double twovleg_time = 0.0;
      double flight_time  = 0.0;
      double speed        = 0.0;
      double max_speed    = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(swing_time));
        a->Visit(MJ_NVP(onevleg_time));
        a->Visit(MJ_NVP(twovleg_time));
        a->Visit(MJ_NVP(flight_time));
        a->Visit(MJ_NVP(speed));
        a->Visit(MJ_NVP(max_speed));
      }
    };

    Trot trot;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(idle_count));
      a->Visit(MJ_NVP(legs));
      a->Visit(MJ_NVP(vlegs));
      a->Visit(MJ_NVP(next_step_vleg));
      a->Visit(MJ_NVP(stance_elapsed_s));
      a->Visit(MJ_NVP(last_swing_v_R));
      a->Visit(MJ_NVP(travel_distance));
      a->Visit(MJ_NVP(trot));
    }
  };

  Walk walk;

  // Estado de la rutina estática activa. Solo válido en kRoutine.
  RoutineState routine;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs_B));
    a->Visit(MJ_NVP(robot));
    a->Visit(MJ_NVP(stand_up));
    a->Visit(MJ_NVP(rest));
    a->Visit(MJ_NVP(situp));
    a->Visit(MJ_NVP(jump));
    a->Visit(MJ_NVP(walk));
    a->Visit(MJ_NVP(routine));
  }
};

inline QuadrupedState::Leg operator*(const Sophus::SE3d& pose_AB,
                                     const QuadrupedState::Leg& rhs_B) {
  auto result_A = rhs_B;
  result_A.position = pose_AB * rhs_B.position;
  result_A.velocity = pose_AB.so3() * rhs_B.velocity;
  result_A.force_N  = pose_AB.so3() * rhs_B.force_N;
  return result_A;
}

}  // namespace mech
}  // namespace mjmech

namespace mjlib {
namespace base {

template <>
struct IsEnum<mjmech::mech::QuadrupedState::StandUp::Mode> {
  static constexpr bool value = true;
  using M = mjmech::mech::QuadrupedState::StandUp::Mode;
  static inline std::map<M, const char*> map() {
    return {
      { M::kPrepositioning, "prepositioning" },
      { M::kStanding,       "standing"       },
      { M::kDone,           "done"           },
    };
  }
};

template <>
struct IsEnum<mjmech::mech::QuadrupedState::Jump::Mode> {
  static constexpr bool value = true;
  using M = mjmech::mech::QuadrupedState::Jump::Mode;
  static inline std::map<M, const char*> map() {
    return {
      { M::kLowering,   "lowering"   },
      { M::kPushing,    "pushing"    },
      { M::kRetracting, "retracting" },
      { M::kFalling,    "falling"    },
      { M::kLanding,    "landing"    },
      { M::kDone,       "done"       },
    };
  }
};

template <>
struct IsEnum<mjmech::mech::QuadrupedState::Situp::Mode> {
  static constexpr bool value = true;
  using M = mjmech::mech::QuadrupedState::Situp::Mode;
  static inline std::map<M, const char*> map() {
    return {
      { M::kLowering, "lowering" },
      { M::kPulling,  "pulling"  },
      { M::kDone,     "done"     },
    };
  }
};

template <>
struct IsEnum<mjmech::mech::QuadrupedState::Walk::VLeg::Mode> {
  static constexpr bool value = true;
  using M = mjmech::mech::QuadrupedState::Walk::VLeg::Mode;
  static inline std::map<M, const char*> map() {
    return {
      { M::kStance, "stance" },
      { M::kSwing,  "swing"  },
    };
  }
};

}  // namespace base
}  // namespace mjlib