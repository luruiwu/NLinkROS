#include "ncommon.h"
#include <array>
#include <vector>

namespace NCommon {

int stringToHex(const char *str, char *out) {
  int outLength = 0;
  int cnt = 0;
  uint8_t high = 0, low = 0;
  char current = 0;
  uint8_t value = 0;
  uint8_t isHighValid = 0;
  while (current = str[cnt]) {
    ++cnt;
    if (current >= '0' && current <= '9') {
      value = current - '0';
    } else if (current >= 'a' && current <= 'f') {
      value = current - 'a' + 10;
    } else if (current >= 'A' && current <= 'F') {
      value = current - 'A' + 10;
    } else {
      continue;
    }

    if (!isHighValid) {
      high = value;

      isHighValid = 1;
    } else {
      low = value;

      out[outLength] = high << 4 | low;
      ++outLength;
      isHighValid = 0;
    }
  }

  return outLength;
}

std::string StringToHex(const std::string &src) {
  std::vector<char> data(src.size() / 2);
  stringToHex(src.data(), data.data());
  return std::string(data.data(), data.size());
}

} // namespace NCommon
