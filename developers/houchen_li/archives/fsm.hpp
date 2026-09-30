/**
 * @file fsm.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief This FSM is derived from digint's tinyfsm project (https://github.com/digint/tinyfsm).
 * @version 0.1
 * @date 2023-12-12
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <functional>
#include <stack>
#include <vector>

namespace basjoo::common {

namespace detail {

template <typename S>
class StateInstance final {
  public:
    using value_type = S;
    using type = StateInstance<S>;
    static S value;
};

template <typename S>
typename StateInstance<S>::value_type StateInstance<S>::value;

} // namespace detail

class Event {
  public:
    Event() noexcept = default;
    Event(const Event& other) noexcept = delete;
    auto operator=(const Event& other) noexcept -> Event& = delete;
    Event(Event&& other) noexcept = delete;
    auto operator=(Event&& other) noexcept -> Event& = delete;
    virtual ~Event() noexcept = 0;
};

inline Event::~Event() noexcept = default;

template <typename F>
class Fsm {
  public:
    Fsm() noexcept = default;
    Fsm(const Fsm& other) noexcept = delete;
    auto operator=(const Fsm& other) noexcept -> Fsm& = delete;
    Fsm(Fsm&& other) noexcept = delete;
    auto operator=(Fsm&& other) noexcept -> Fsm& = delete;
    virtual ~Fsm() noexcept = 0;

    template <typename S>
    static constexpr auto state() -> S& {
        return detail::StateInstance<S>::value;
    }

    static auto initialize() -> void;

    static auto reset() -> void {}

    static auto enter() -> void {
        m_state_stack.top()->entry();
        return;
    }

    static auto start() -> void {
        initialize();
        enter();
        return;
    }

    template <typename E>
    static auto dispatch(const E& event) -> void {
        m_state_stack.top()->react(event);
        return;
    }

    static auto getCurrentState() -> F* { return m_state_stack.top(); }

  protected:
    template <typename S>
    static auto pushState() -> void {
        m_state_stack.push(&detail::StateInstance<S>::value);
        m_state_stack.top()->entry();
        return;
    }

    template <typename S>
    static auto pushState(const std::function<auto()->void>& action_function) -> void {
        action_function();
        m_state_stack.push(&detail::StateInstance<S>::value);
        m_state_stack.top()->entry();
        return;
    }

    template <typename S>
    static auto pushState(
        const std::function<auto()->void>& action_function,
        const std::function<auto()->bool>& condition_function
    ) -> void {
        if (condition_function()) {
            pushState<S>(action_function);
        }
        return;
    }

    static auto popState() -> void {
        m_state_stack.top()->exit();
        m_state_stack.pop();
        return;
    }

  private:
    static std::stack<F*, std::vector<F*>> m_state_stack;
};

template <typename F>
inline Fsm<F>::~Fsm() noexcept = default;

template <typename F>
std::stack<F*, std::vector<F*>> Fsm<F>::m_state_stack{};

template <typename... FF>
class FsmList;

template <>
class FsmList<> {
  public:
    FsmList() noexcept = default;
    FsmList(const FsmList& other) noexcept = delete;
    auto operator=(const FsmList& other) noexcept -> FsmList& = delete;
    FsmList(FsmList&& other) noexcept = delete;
    auto operator=(FsmList&& other) noexcept -> FsmList& = delete;
    virtual ~FsmList() noexcept = default;

    static auto initialize() -> void {}

    static auto reset() -> void {}

    static auto enter() -> void {}

    template <typename E>
    static auto dispatch([[maybe_unused]] const E& event) -> void {}
};

template <typename F, typename... FF>
class [[nodiscard]] FsmList<F, FF...> {
  public:
    FsmList() noexcept = default;
    FsmList(const FsmList& other) noexcept = delete;
    auto operator=(const FsmList& other) noexcept -> FsmList& = delete;
    FsmList(FsmList&& other) noexcept = delete;
    auto operator=(FsmList&& other) noexcept -> FsmList& = delete;
    virtual ~FsmList() noexcept = default;

    static auto initialize() -> void {
        Fsm<F>::initialize();
        FsmList<FF...>::initialize();
        return;
    }

    static auto reset() -> void {}

    static auto enter() -> void {
        Fsm<F>::enter();
        FsmList<FF...>::enter();
        return;
    }

    static auto start() -> void {
        initialize();
        enter();
        return;
    }

    template <typename E>
    static auto dispatch(const E& event) -> void {
        Fsm<F>::template dispatch<E>(event);
        FsmList<FF...>::template dispatch<E>(event);
        return;
    }
};

template <typename... SS>
class StateList;

template <>
class StateList<> {
  public:
    StateList() noexcept = default;
    StateList(const StateList& other) noexcept = delete;
    auto operator=(const StateList& other) noexcept -> StateList& = delete;
    StateList(StateList&& other) noexcept = delete;
    auto operator=(StateList&& other) noexcept -> StateList& = delete;
    virtual ~StateList() noexcept = default;

    static auto reset() -> void {}
};

template <typename S, typename... SS>
class [[nodiscard]] StateList<S, SS...> {
  public:
    StateList() noexcept = default;
    StateList(const StateList& other) noexcept = delete;
    auto operator=(const StateList& other) noexcept -> StateList& = delete;
    StateList(StateList&& other) noexcept = delete;
    auto operator=(StateList&& other) noexcept -> StateList& = delete;
    virtual ~StateList() noexcept = default;

    static auto reset() -> void {
        S::reset();
        StateList<SS...>::reset();
        return;
    }
};

} // namespace basjoo::common

#define FSM_INITIAL_STATE(FSM, STATE)                                               \
    template <>                                                                     \
    auto ::basjoo::common::Fsm<FSM>::initialize()->void {                           \
        while (!m_state_stack.empty()) {                                            \
            m_state_stack.pop();                                                    \
        }                                                                           \
        m_state_stack.push(&::basjoo::common::detail::StateInstance<STATE>::value); \
        return;                                                                     \
    }
