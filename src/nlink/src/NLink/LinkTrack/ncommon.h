#ifndef LINKTRACKNCOMMON_H
#define LINKTRACKNCOMMON_H

#include "../ndatatype.h"
#include <cassert>

namespace LinkTrack {

namespace DataType {

typedef NDataInt<uint16_t, 1000> SupplyVoltage;
typedef NDataInt<NInt24, 1000> Distance;
typedef NVector3D<NInt24, 1000> Position;
typedef NVector3D<NInt24, 10000> Velocity;
typedef NVector3D<int16_t, 100> Euler;
typedef NVector3D<uint8_t, 100> EOP;

typedef NDataInt<uint8_t, -2> RSSI;

typedef NVector3D<float> GYRO;
typedef NVector3D<float> ACC;

enum Role : uint8_t {
  kRoleNon = 0xff,  //!< 异常值
  kRoleNode = 0x00, //!< 节点
  kRoleAnchor,      //!< 基站
  kRoleTag,         //!< 标签
  kRoleConsole,     //!< 监视器
  kRoleDTMaster,    //!< DT 主机
  kRoleDTSlave,     //!< DT 从机

  kRoleCount, //!< 角色数量
};

enum Mode : int8_t {
  kModeNon = -1,

  kModeLP0,
  kModeLP1,
  kModeLP2,
  kModeDT0,
  kModeDT1,
  kModeDT2,
  kModeDT3,
  kReserved1,
  kModeCM,
  kModeAHRS,
  kModeDR,

  kModeCount,
};

} // namespace DataType

extern const std::vector<std::string> kProductNames;

extern const std::vector<int> kBaudRates;
} // namespace LinkTrack

#endif // LINKTRACKNCOMMON_H
