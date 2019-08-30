#ifndef LINKTRACKPARAMETER_H
#define LINKTRACKPARAMETER_H

#include "../../NProtocol/nprotocolbase.h"
#include "../../NProtocol/nprotocolitembase.h"
#include "ncommon.h"

namespace LinkTrack {

class NParameter : public NProtocolBase {
public:
  explicit NParameter();

  void unpackData(const std::string &byteArray) override;

  std::string updateCommand() override;

#pragma pack(1)
  struct Frame_t : public FrameBase<128> {
    char header[2]{0x54, 0x00};
    struct Mix {
      uint8_t readParam : 1;          // WO
      uint8_t commandType : 2;        // WO
      uint8_t restartCurrentNode : 1; // WO
      uint8_t restoreFactory : 1;
      uint8_t readCM_AppParam : 1;
      uint8_t : 2;
      Mix() { *reinterpret_cast<uint8_t *>(this) = 0; }
    } mix{};
    DataType::Role role = DataType::kRoleNon; // RW
    uint8_t model = 0;                        // RW
    NInt24 baudRate;                          // RW
    uint8_t systemNum = 0;                    // RW
    uint8_t id = 0;                           // RW
    uint16_t refreshRate = 0;                 // RW
    uint8_t systemID = 0;                     // RW
    uint8_t reservedWithFF1[1] = {0xff};

    struct OnOffControl {
      uint8_t UARTLed : 1;   // RW
      uint8_t UWBLed : 1;    // RW
      uint8_t fusionIMU : 1; // RW
      uint8_t : 5;
      OnOffControl() { *reinterpret_cast<uint8_t *>(this) = 0; }
    } onOffControl{};
    uint8_t reservedWithFF2[1] = {0xff};
    uint8_t filterFactor = 0; // RW
    struct AppMode {
      DataType::Mode running : 4;
      DataType::Mode local : 4;
      bool operator!=(const AppMode &source) {
        return source.running != running || source.local != local;
      }
    } appMode{DataType::kModeLP0, DataType::kModeLP0};
    uint8_t appRole = 0;       // RW
    uint8_t protocolIndex = 0; // RW
    uint8_t txGain{};
    uint8_t reservedWithFF3[3] = {0xff, 0xff};
    uint8_t nodeCapacity{};
    uint8_t reservedWithFF4[2]{0xff, 0xff};
    uint32_t systemTime = 0; // RO
    uint8_t reservedWithFF5[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
    uint8_t anchorGroupIndex = 0;

    struct Coordinates {
      static const size_t skMaxAnchorsPerFrame = 10;
      char value[3 * 3 * skMaxAnchorsPerFrame]{};
      bool operator!=(const Coordinates &source) {
        return 0 != memcmp(source.value, value, sizeof(Coordinates));
      }
    } coordinates;

    uint8_t checkSum = 0;
  };
  static_assert((sizeof(Frame_t) == Frame_t::skLength), "Error");

#pragma pack()
  Frame_t data() const;

private:
  Frame_t data_;
};

} // namespace LinkTrack
#endif // LINKTRACKPARAMETER_H
