#ifndef TOOL_TERMINAL_COLOR_H
#define TOOL_TERMINAL_COLOR_H

namespace tool::terminal_color {

// ANSI 终端颜色码。彩色文本末尾追加 kReset，恢复终端默认样式。
inline constexpr char kReset[] = "\033[0m";
inline constexpr char kGreen[] = "\033[1;32m";
inline constexpr char kYellow[] = "\033[1;33m";
inline constexpr char kRed[] = "\033[1;31m";
inline constexpr char kCyan[] = "\033[1;36m";
inline constexpr char kMagenta[] = "\033[1;35m";

}  // namespace tool::terminal_color

#endif  // TOOL_TERMINAL_COLOR_H
