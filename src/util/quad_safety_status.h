#ifndef QUAD_SAFETY_STATUS_H
#define QUAD_SAFETY_STATUS_H

namespace game_engine {
    enum class QuadSafetyStatus {
        OK = 0,
        PRIMARY = 1,
        ALTERNATE = 2,
        CONTINGENT = 3,
        EMERGENCY = 4,
    };
}

#endif
