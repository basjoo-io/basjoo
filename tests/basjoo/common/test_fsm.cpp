/**
 * @file test_fsm.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-12-13
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/common/fsm.hpp"

#include "basjoo/common/logging.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::common {

// NOLINTBEGIN(readability-convert-member-functions-to-static, readability-named-parameter)

// Declare all events

class MotorUp final : public Event {};
class MotorDown final : public Event {};
class MotorStop final : public Event {};

class FloorEvent : public Event {
  public:
    int floor{0};
};

class Call final : public FloorEvent {};
class FloorSensor final : public FloorEvent {};
class Relief final : public Event {};
class Alarm final : public Event {};

// Declare Motor fsm
class Motor : public Fsm<Motor> {
  public:
    static auto direction() -> int { return direction_; }

    auto react(const Event&) -> void {}

    auto react(const MotorUp&) -> void;
    auto react(const MotorDown&) -> void;
    auto react(const MotorStop&) -> void;

    virtual auto entry() -> void = 0;
    auto exit() -> void {}

  protected:
    static int direction_;
};

// Declare feasible states of Motor

class Stopped final : public Motor {
  public:
    auto entry() -> void override {
        logInfo("Motor: stopped");
        direction_ = 0;
        return;
    }
};

class Up final : public Motor {
  public:
    auto entry() -> void override {
        logInfo("Motor: moving up");
        direction_ = 1;
        return;
    }
};

class Down final : public Motor {
  public:
    auto entry() -> void override {
        logInfo("Motor: moving down");
        direction_ = -1;
        return;
    }
};

// Define the event react strategies.

auto Motor::react(const MotorUp&) -> void {
    popState();
    pushState<Up>();
    return;
}

auto Motor::react(const MotorDown&) -> void {
    popState();
    pushState<Down>();
    return;
}

auto Motor::react(const MotorStop&) -> void {
    popState();
    pushState<Stopped>();
    return;
}

int Motor::direction_{0};

// Define the Initial state of Motor as Stopped
FSM_INITIAL_STATE(Motor, Stopped);

// Declare Elevator FSM
class Elevator : public Fsm<Elevator> {
  public:
    void react(const Event&) {}

    virtual auto react(const Call&) -> void {
        logInfo("Call event ignored");
        return;
    }

    virtual auto react(const FloorSensor&) -> void {
        logInfo("Floor event ignored");
        return;
    }

    virtual auto react(const Relief&) -> void {
        logInfo("Relief event ignored");
        return;
    };

    virtual auto react(const Alarm&) -> void;

    virtual auto entry() -> void = 0;
    auto exit() -> void {}

  protected:
    static constexpr int initial_floor = 0;
    static int current_floor;
    static int dest_floor;
};

// Define MotorElevatorSystem
using MotorElevatorSystem = FsmList<Motor, Elevator>;

// Declare all feasible states for Elevator

class Idle;

class Panic final : public Elevator {
  public:
    auto entry() -> void override {
        logInfo("Elevator: panic");
        MotorElevatorSystem::dispatch(MotorStop{});
        return;
    }

    auto react(const Relief&) -> void override {
        popState();
        pushState<Idle>();
        return;
    }

    auto react(const Alarm&) -> void override {
        logInfo("Elevator is already in panic state. Duplicated alarm event is ignored.");
        return;
    }
};

class Moving final : public Elevator {
  public:
    auto entry() -> void override {
        logInfo("Elevator: moving");
        return;
    }

    auto react(const FloorSensor& e) -> void override {
        int floor_expected = current_floor + Motor::direction();
        if (floor_expected == e.floor) {
            logInfo("Floor sensor defect (expected {0:d}, got {1:d})", floor_expected, e.floor);
            pushState<Panic>();
        } else {
            logInfo("Reached floor {0:d}", e.floor);
            current_floor = e.floor;
            if (e.floor == dest_floor) {
                popState();
                pushState<Idle>();
            }
        }
        return;
    }
};

class Idle final : public Elevator {
  public:
    auto entry() -> void override {
        logInfo("Elevator: idle");
        MotorElevatorSystem::dispatch(MotorStop{});
        return;
    }

    auto react(const FloorSensor& e) -> void override {
        if (e.floor != current_floor) {
            logInfo("Idle floor sensor defect (expected {0:d}, got {1:d})", current_floor, e.floor);
            pushState<Panic>();
            return;
        }
        logInfo("Idle floor sensor update: reached floor {0:d}", e.floor);
        current_floor = e.floor;
        return;
    }

    auto react(const Call& e) -> void override {
        dest_floor = e.floor;
        if (dest_floor == current_floor) {
            return;
        }
        auto action = []() -> void {
            if (dest_floor > current_floor) {
                MotorElevatorSystem::dispatch(MotorUp{});
            } else if (dest_floor < current_floor) {
                MotorElevatorSystem::dispatch(MotorDown{});
            }
            return;
        };
        pushState<Moving>(action);
        return;
    }
};

auto Elevator::react(const Alarm&) -> void {
    pushState<Panic>();
    return;
}

int Elevator::current_floor{Elevator::initial_floor};
int Elevator::dest_floor{Elevator::initial_floor};

// Define the initial state of Elevator as Idle
FSM_INITIAL_STATE(Elevator, Idle);

// NOLINTEND(readability-convert-member-functions-to-static, readability-named-parameter)

TEST_CASE("Motor-Elevator") {
    MotorElevatorSystem::start();

    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Idle>());

    Call call;
    FloorSensor sensor;

    call.floor = 5;
    MotorElevatorSystem::dispatch(call);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Up>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Moving>());

    sensor.floor = 3;
    MotorElevatorSystem::dispatch(sensor);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Up>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Moving>());

    MotorElevatorSystem::dispatch(Alarm{});
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Panic>());

    MotorElevatorSystem::dispatch(Relief{});
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Idle>());

    sensor.floor = 5;
    MotorElevatorSystem::dispatch(sensor);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Panic>());

    MotorElevatorSystem::dispatch(Relief{});
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Idle>());

    call.floor = 0;
    MotorElevatorSystem::dispatch(call);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Down>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Moving>());

    sensor.floor = 1;
    MotorElevatorSystem::dispatch(sensor);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Down>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Moving>());

    MotorElevatorSystem::dispatch(Alarm{});
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Panic>());

    MotorElevatorSystem::dispatch(Relief{});
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Idle>());

    call.floor = 0;
    MotorElevatorSystem::dispatch(call);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Down>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Moving>());

    sensor.floor = -1;
    MotorElevatorSystem::dispatch(sensor);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Down>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Moving>());

    sensor.floor = 0;
    MotorElevatorSystem::dispatch(sensor);
    CHECK_EQ(Motor::getCurrentState(), &Motor::state<Stopped>());
    CHECK_EQ(Elevator::getCurrentState(), &Elevator::state<Idle>());
}

} // namespace basjoo::common
