#include "diagnostic_reader.hpp"

#include <cstring>

#include "diagnostic_text.hpp"
#include "diagnostics.hpp"
#include "diagnostics_config.hpp"

namespace vehicle {
namespace diagnostics {
namespace reader {
namespace {

// cursor记录已确认事件的进度。
std::uint32_t cursor{};
// replay_first_fault为true时优先返回首故障。
bool replay_first_fault=true;
// text保存最近一次读取渲染的UTF-8文本。
char text[DiagnosticTextCapacity]{"none"};

// FindCurrent()定位当前待读事件：首故障优先，否则按游标取最早留存事件。
bool FindCurrent(Event &out)
{
    if (replay_first_fault && ReadFirstFault(out)) { return true; }
    return ReadEvent(cursor,out);
}

} // namespace

void Rewind()
{
    cursor=0;
    replay_first_fault=true;
}

const char *Text()
{
    Event event{};
    if (!FindCurrent(event)) { std::strcpy(text,"none"); return text; }
    // 只有首故障事件带bit1标记；读到其它事件后不再重放。
    if (!(event.flags & 2)) { replay_first_fault=false; }
    FormatEvent(event,text,sizeof(text));
    return text;
}

bool Ack(std::uint32_t seq)
{
    Event event{};
    if (!seq || !FindCurrent(event) || seq!=event.event_seq) { return false; }
    // 首故障先重放，之后从环中最早保留事件开始。
    // 独立首故障不能跳过较早非致命事件。
    if (replay_first_fault) { replay_first_fault=false; }
    else { cursor=seq; }
    return true;
}

} // namespace reader
} // namespace diagnostics
} // namespace vehicle
