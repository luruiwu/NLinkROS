#ifndef TOFSENSENFRAME0_H
#define TOFSENSENFRAME0_H

#include "../../NProtocol/nprotocolbase.h"
#include "ncommon.h"

namespace TOFSense {

class NFrame0 : public NProtocolBase {
public:
  explicit NFrame0();

  void unpackData(const std::string &byteArray) override;
#pragma pack(1)
  struct Frame_t : public FrameBase<16> {
    char header[2]{0x57, 0x00};
    uint8_t reserved0[1]{};
    uint8_t id{};
    uint32_t systemTime{};
    DataType::Distance distance{};
    uint8_t distanceStatus{};
    uint16_t signalStrength{};
    uint8_t reserved1[1]{};
    uint8_t checkSum{};
  };
  static_assert(sizeof(Frame_t) == Frame_t::skLength, "Error");
#pragma pack()
  Frame_t data() const;

private:
  Frame_t data_;
};

} // namespace TOFSense

#endif // TOFSENSENFRAME0_H
