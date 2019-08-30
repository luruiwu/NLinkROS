#ifndef NPROTOCOLNCOMMON_H
#define NPROTOCOLNCOMMON_H

#include <string>
namespace NCommon {

// QString HexToString(const QByteArray &data, char separator = ' ');

int stringToHex(const char *str, char *out);

std::string StringToHex(const std::string &src);

} // namespace NCommon

#endif // NPROTOCOLNCOMMON_H
