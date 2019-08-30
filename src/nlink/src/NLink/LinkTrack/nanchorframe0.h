#ifndef LINKTRACKKANCHORFRAME0_H
#define LINKTRACKANCHORFRAME0_H

#include "../../NProtocol/nprotocolbase.h"
#include "ncommon.h"

namespace LinkTrack {

class NAnchorFrame0 : public NProtocolBase {
public:
  explicit NAnchorFrame0();

  void unpackData(const std::string &byteArray) override;
#pragma pack(1)
  struct Tag_t {
    uint8_t id{};
    uint8_t reserved{};
    DataType::Position position{};
    struct Dis_t {
      uint8_t id{};
      DataType::Distance distance{};
    };
    Dis_t anchors[4]{};
  };

  struct Frame_t {
    char header[2]{0x55, 0x00};
    Tag_t tags[30];
    uint8_t reserved0[75]{};
    DataType::SupplyVoltage supplyVoltage{};
    uint32_t systemTime{};
    uint8_t id{};
    uint8_t reserved1[1]{};
    char check[1]{static_cast<char>(0xee)};
  };
  static_assert((sizeof(Frame_t) == 896), "Error");

#pragma pack()
  Frame_t data() const;

private:
  Frame_t data_;
};

} // namespace LinkTrack
#endif // LINKTRACKANCHORFRAME0_H
