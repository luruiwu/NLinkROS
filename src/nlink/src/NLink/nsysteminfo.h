#ifndef NLINKSYSTEMINFO_H
#define NLINKSYSTEMINFO_H

#include "../NProtocol/nprotocolbase.h"
#include "../NProtocol/nprotocolitembase.h"
#include "ndatatype.h"
#include <sstream>

namespace NLink {

class NSystemInfo : public NProtocolBase {
public:
  explicit NSystemInfo();

  void unpackData(const std::string &byteArray) override;

  std::string updateCommand() override;

  std::string readCommand();

private:
  std::string readCommand_;

  template <size_t nums> struct VersionBase {
    //      static const int skNums = nums;
    std::string string() const {
      auto *num = static_cast<const uint8_t *>(this);
      std::ostringstream os;
      int i = 0;
      os << num[i++];
      for (; i < nums; ++i) {
        os << '.' << num[i];
      }
      return os.str();
    }
  };

#pragma pack(1)
  struct Frame_t : public FrameBase<32> {
    char header[2]{0x52, 0x00};
    struct Mix {
      uint8_t readParam : 1;
      uint8_t : 7;
      Mix() { *reinterpret_cast<uint8_t *>(this) = 0; }
    } mix{};

    struct ProductVersion : public VersionBase<2> {
      uint8_t low{};
      uint8_t high{};
    } productVersion{};
    struct HardwareVersion : public VersionBase<2> {
      uint8_t low{};
      uint8_t high{};
    } hardwareVersion{};
    struct FirmwareVersion : public VersionBase<4> {
      uint8_t Beta{};
      uint8_t low{};
      uint8_t middle{};
      uint8_t high{};
    } firmwareVersion{};

    uint8_t reservedWithFF0[8]{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    NInt24 baudRate;
    uint8_t reserved1[2]{0xff, 0xff};
    //    DataType::Role role = DataType::kRoleNon; // RW
    //    uint8_t id = 0;                           // RW
    uint32_t systemTime = 0;
    uint8_t reserved2[3];
    uint8_t checkSum = 0;
  } data_;

  static_assert((sizeof(Frame_t) == Frame_t::skLength), "Error");

#pragma pack()
};

} // namespace NLink

#endif // NLINKSYSTEMINFO_H
