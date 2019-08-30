#include "init.h"
#include "../NLink/TOFSense/nframe0.h"
#include "../NProtocol/ncommon.h"
#include "../NProtocol/nframeextraction.h"
#include <nlink/tofsense_frame0.h>

namespace TOFSense {

Init::Init(NFrameExtraction *frameExtraction) {
  initFrame0(frameExtraction);

  //  test(frameExtraction);
}

void Init::initFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NFrame0;
  frameExtraction->appendProtocol(protocol);
  protocol->setDataHandle([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nodeHandle;
      publishers_[protocol] = nodeHandle.advertise<nlink::tofsense_frame0>(
          "nlink_tofsense_frame0", 50);
    }
    auto data = protocol->data();
    nlink::tofsense_frame0 msgData;

    msgData.id = data.id;
    msgData.systemTime = data.systemTime;
    msgData.distance = data.distance();
    msgData.distanceStatus = data.distanceStatus;
    msgData.signalStrength = data.signalStrength;

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::test(NFrameExtraction *frameExtraction) {
  frameExtraction->unpackData(
      NCommon::StringToHex("57 00 ff 00 9e 8f 00 00 ad 08 00 00 03 00 ff 3a"));
}

} // namespace TOFSense
