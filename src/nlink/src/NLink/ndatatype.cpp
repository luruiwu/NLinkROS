#include "ndatatype.h"
#include <cassert>
//#include <QDebug>

const bool kIsLittleEndianFormat = [] {
  uint32_t data = 1;
  char *p = reinterpret_cast<char *>(&data);
  bool isLittle = *p == 1;
  //  if (!isLittle)
  //    qFatal("CPU is in big endian format,current library\
// only support little endian format,contact Nooploop company for updates");
  assert(isLittle);
  return isLittle;
}();
