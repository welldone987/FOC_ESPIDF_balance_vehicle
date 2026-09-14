#include "diagnostic_text.hpp"

#include <cstdio>
#include <cstring>

namespace vehicle {
namespace diagnostics {
namespace {

// FormatNumber()把有效数值按"%g"加单位后缀写入缓冲；无效时写NULL。
void FormatNumber(char *out, std::size_t capacity, float number, bool valid, const char *unit)
{
    if (!valid) { std::strcpy(out,"NULL"); return; }
    std::snprintf(out,capacity,"%g%s",static_cast<double>(number),unit);
}

} // namespace

int FormatEvent(const Event &event, char *buffer, std::size_t capacity)
{
    const auto &e=event.error;
    const char *file=e.file ? e.file : "?";
    // 只保留文件名部分，去掉构建路径。
    const char *base=std::strrchr(file,'/');
    if (base) { file=base+1; }
    base=std::strrchr(file,'\\');
    if (base) { file=base+1; }
    // raw仅在非0且不同于code时输出，其余情况占位NULL。
    char raw[16]{};
    if (e.raw_code != 0 && e.raw_code != static_cast<std::int32_t>(e.code)) {
        std::snprintf(raw,sizeof(raw),"%ld",static_cast<long>(e.raw_code));
    } else { std::strcpy(raw,"NULL"); }
    // value/threshold按valid_fields位输出，并追加该点的单位后缀。
    const char *unit=PointUnit(e.point_id);
    char value[32]{}, threshold[32]{}, channel[16]{};
    FormatNumber(value,sizeof(value),e.value,(e.valid_fields & ErrorValue)!=0,unit);
    FormatNumber(threshold,sizeof(threshold),e.threshold,(e.valid_fields & ErrorThreshold)!=0,unit);
    if (e.channel >= 0) { std::snprintf(channel,sizeof(channel),"%d",static_cast<int>(e.channel)); }
    else { std::strcpy(channel,"NULL"); }
    return std::snprintf(buffer,capacity,
        "seq=%lu flags=%u point=0x%04x code=%ld(%s) raw=%s at=%s:%lu value=%s threshold=%s ch=%s",
        static_cast<unsigned long>(event.event_seq),event.flags,static_cast<unsigned>(e.point_id),
        static_cast<long>(e.code),esp_err_to_name(e.code),
        raw,file,static_cast<unsigned long>(e.line),value,threshold,channel);
}

} // namespace diagnostics
} // namespace vehicle
