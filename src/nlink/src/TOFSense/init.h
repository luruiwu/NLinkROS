#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

#include "../NProtocol/nprotocolbase.h"
#include <ros/ros.h>
#include <unordered_map>
class NFrameExtraction;

namespace TOFSense {

class Init {
public:
  explicit Init(NFrameExtraction *frameExtraction);
  //  ~Init();

private:
  void initFrame0(NFrameExtraction *frameExtraction);

  void test(NFrameExtraction *frameExtraction);
  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
};

} // namespace TOFSense
#endif // TOFSENSEINIT_H
