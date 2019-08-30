#ifndef NFRAMEEXTRACTION_P_H
#define NFRAMEEXTRACTION_P_H

#include "nprotocolbase.h"
#include <string>
#include <vector>

class NFrameExtraction;
class NFrameExtraction_P {
public:
  NFrameExtraction_P() = default;
  ~NFrameExtraction_P();

  std::vector<NProtocolBase *> protocols_;

  std::string prevTempArray_;
  int tempArrayIndex_ = 0;
  size_t lackIndex_ = 0;

  typedef struct {
    size_t protocolIndex;
    size_t headerIndex;
    size_t protocolBytes;
  } SortInfo;

  std::vector<SortInfo> sortInfos_;

  bool isAnalysingData_ = false;
};

#endif // NFRAMEEXTRACTION_P_H
