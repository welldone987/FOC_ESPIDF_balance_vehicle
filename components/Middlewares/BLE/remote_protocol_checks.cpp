#include "remote_protocol.hpp"

// 构建期执行真实解析器和状态机；不生成测试任务，不访问硬件。
namespace vehicle::ble {
namespace {
constexpr bool valid(std::string_view text, bool legacy = false)
{
    RemoteCommand command{};
    return parseCommand(text, legacy, command);
}
static_assert(valid("D,65535,-100,100"));
static_assert(valid("A,0") && valid("S,12") && valid("E,12"));
static_assert(valid("100,-100\n", true));
static_assert(!valid("") && !valid("D,1,101,0") && !valid("D,-1,0,0"));
static_assert(!valid("D,65536,0,0") && !valid("D,999999999999,0,0"));
static_assert(!valid("D,1,0,0,1") && !valid("D,1,,0") && !valid("A,1,0"));
static_assert(!valid("D,1,0,0\n") && !valid(" D,1,0,0"));
static_assert(!valid(std::string_view("D,1,0,0\0X", 9)));
static_assert(!valid("0,1oops", true) && !valid("0,", true));
static_assert(newerSequence(0, 65535) && !newerSequence(5, 5));
static_assert(!newerSequence(4, 5) && !newerSequence(32768, 0));

constexpr RemoteState armed()
{
    RemoteState state{};
    remoteConnection(state, true);
    stepRemote(state, 0);
    acceptCommand(state, {'D', 1, 0, 0, false}, 1);
    stepRemote(state, 1);
    acceptCommand(state, {'A', 2, 0, 0, false}, 2);
    stepRemote(state, 2);
    return state;
}
static_assert(armed().mode == RemoteMode::active);

constexpr bool timeoutAndNoAutomaticResume()
{
    auto state = armed();
    stepRemote(state, 300001);
    if (state.mode != RemoteMode::active) { return false; }
    stepRemote(state, 300002);
    if (state.mode != RemoteMode::timeout) { return false; }
    acceptCommand(state, {'D', 3, 0, 50, false}, 300003);
    stepRemote(state, 300003);
    return state.mode == RemoteMode::timeout;
}
static_assert(timeoutAndNoAutomaticResume());

constexpr bool latePacketCannotHideTimeout()
{
    auto state = armed();
    acceptCommand(state, {'D', 3, 0, 50, false}, 300002);
    stepRemote(state, 300002);
    return state.mode == RemoteMode::timeout;
}
static_assert(latePacketCannotHideTimeout());

constexpr bool duplicateDoesNotRenew()
{
    auto state = armed();
    const bool accepted = acceptCommand(state, {'D', 2, 0, 50, false}, 200000);
    stepRemote(state, 300002);
    return !accepted && state.mode == RemoteMode::timeout && state.received_us == 2;
}
static_assert(duplicateDoesNotRenew());

constexpr bool stopCannotBeOverwritten()
{
    auto state = armed();
    acceptCommand(state, {'S', 3, 0, 0, false}, 3);
    acceptCommand(state, {'D', 4, 0, 50, false}, 4);
    stepRemote(state, 4);
    return state.mode == RemoteMode::stopped && state.applied_sequence == 2;
}
static_assert(stopCannotBeOverwritten());

constexpr bool emergencySurvivesReconnect()
{
    auto state = armed();
    acceptCommand(state, {'E', 1, 0, 0, false}, 3);
    if (acceptCommand(state, {'D', 3, 0, 50, false}, 4)) { return false; }
    remoteConnection(state, false);
    remoteConnection(state, true);
    stepRemote(state, 5);
    return state.mode == RemoteMode::emergency;
}
static_assert(emergencySurvivesReconnect());

constexpr bool reconnectRequiresArmAndBlocksLegacy()
{
    auto state = armed();
    if (acceptCommand(state, {'D', 0, 0, 50, true}, 3)) { return false; }
    remoteConnection(state, false);
    stepRemote(state, 4);
    if (state.mode != RemoteMode::disconnected) { return false; }
    remoteConnection(state, true);
    stepRemote(state, 5);
    acceptCommand(state, {'D', 0, 0, 50, false}, 6);
    stepRemote(state, 6);
    return state.mode == RemoteMode::idle &&
        !acceptCommand(state, {'A', 1, 0, 0, false}, 7);
}
static_assert(reconnectRequiresArmAndBlocksLegacy());
} // namespace
} // namespace vehicle::ble
