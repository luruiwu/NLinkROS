#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include "../NProtocol/nprotocolbase.h"
#include <ros/ros.h>
#include <unordered_map>

class NFrameExtraction;
namespace LinkTrack {

class Init {
public:
  explicit Init(NFrameExtraction *frameExtraction);

private:
  void initAnchorFrame0(NFrameExtraction *frameExtraction);
  void initTagFrame0(NFrameExtraction *frameExtraction);
  void initNodeFrame0(NFrameExtraction *frameExtraction);
  void initNodeFrame1(NFrameExtraction *frameExtraction);
  void initNodeFrame2(NFrameExtraction *frameExtraction);

  void test(NFrameExtraction *frameExtraction);
  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
};
} // namespace LinkTrack

#endif // LINKTRACKINIT_H
