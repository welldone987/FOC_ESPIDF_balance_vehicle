#include "remote_protocol.hpp"
#include "motion_command.hpp"

// 构建期执行真实解析器和状态机；不生成测试任务，不访问硬件。
namespace vehicle {
namespace ble {
namespace {
constexpr bool valid(std::string_view text, bool legacy = false)
{
    RemoteCommand command{};
    return !legacy && parseCommand(text, command);
}
static_assert(valid("D,65535,-100,100"));
static_assert(!valid("A,0") && !valid("S,12") && !valid("E,12"));
static_assert(!valid("100,-100\n", true));
static_assert(!valid("") && !valid("D,1,101,0") && !valid("D,-1,0,0"));
static_assert(!valid("D,65536,0,0") && !valid("D,999999999999,0,0"));
static_assert(!valid("D,1,0,0,1") && !valid("D,1,,0") && !valid("A,1,0"));
static_assert(!valid("D,1,0,0\n") && !valid(" D,1,0,0"));
static_assert(!valid(std::string_view("D,1,0,0\0X", 9)));
static_assert(!valid("0,1oops", true) && !valid("0,", true));
static_assert(newerSequence(0, 65535) && !newerSequence(5, 5));
static_assert(!newerSequence(4, 5) && !newerSequence(32768, 0));

constexpr control::MotionCommand command{1.0f,0.5f,1000,true};
static_assert(control::freshCommand(command,300999,1000));
static_assert(!control::freshCommand(command,301000,1000));
static_assert(!control::freshCommand(command,999,0));
static_assert(!control::freshCommand(command,1001,1001));
static_assert(!control::freshCommand({},1001,0));
} // namespace
} // namespace ble
} // namespace vehicle
