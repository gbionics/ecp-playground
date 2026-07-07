#pragma once

#include <termios.h>
#include <unistd.h>

namespace actuator_test {

class RawTty {
public:
  RawTty() noexcept : m_active(false) {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &m_saved) != 0) {
      return;
    }
    termios raw = m_saved;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      m_active = true;
    }
  }

  ~RawTty() {
    if (m_active) {
      tcsetattr(STDIN_FILENO, TCSANOW, &m_saved);
    }
  }

  RawTty(const RawTty &) = delete;
  RawTty &operator=(const RawTty &) = delete;

  int try_read() const noexcept {
    unsigned char c = 0;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return (n == 1) ? static_cast<int>(c) : -1;
  }

private:
  termios m_saved{};
  bool m_active;
};

} // namespace actuator_test
