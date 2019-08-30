#ifndef NLINKNDATATYPE_H
#define NLINKNDATATYPE_H

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <string.h>

extern const bool kIsLittleEndianFormat;

template <typename T, int multiply = 1> //历史原因，非类型模板参数不支持浮点型
class NVector3D {
public:
  inline float x() const { return datas_[0] / static_cast<float>(multiply); }
  inline float y() const { return datas_[1] / static_cast<float>(multiply); }
  inline float z() const { return datas_[2] / static_cast<float>(multiply); }
  inline float at(int i) { return datas_[i] / static_cast<float>(multiply); }

private:
  T datas_[3]{};
};

class NInt24 {
public:
  int32_t operator()() const {
    return static_cast<int32_t>(byte_[0] << 8 | byte_[1] << 16 |
                                byte_[2] << 24) /
           256;
  }
  template <typename T> T operator/(T value) const {
    return operator()() / value;
  }

  template <typename T> void operator=(T value) {
    int32_t temp = value;
    memcpy(byte_, &temp, sizeof(byte_));
  }

private:
  uint8_t byte_[3]{};
};

template <typename T, int multiply> class NDataInt {
public:
  float operator()() const { return data_ / static_cast<float>(multiply); }

  template <typename D> void operator=(D value) { data_ = value * multiply; }

private:
  T data_{};
};

template <size_t frameLength, size_t checkSumSize = 1> struct FrameBase {
  static const int skLength = frameLength;
  void updateCheckSum() {
    uint64_t sum = 0;
    static_assert((checkSumSize >= 1 && checkSumSize <= sizeof(sum) &&
                   checkSumSize < frameLength),
                  "FrameBase Size Error");
    auto data = reinterpret_cast<uint8_t *>(this);
    sum = std::accumulate(data, data + frameLength - checkSumSize, sum);
    memcpy(data + frameLength - checkSumSize, &sum, checkSumSize);
  }
  std::string byteArray() const {
    return std::string(reinterpret_cast<const char *>(this), frameLength);
  }
  void update(const void *data) { memcpy(this, data, frameLength); }
};

#endif // NLINKNDATATYPE_H
