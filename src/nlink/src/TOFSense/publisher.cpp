#include "../NProtocol/nframeextraction.h"
#include "../serial/initserial.h"
#include "init.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "TOFSense_Processor");
  ros::NodeHandle nodeHandle;
  NFrameExtraction frameExtraction;
  TOFSense::Init tofsenseInit(&frameExtraction);

  serial::Serial serial;
  initSerial(argc, argv, &serial);

  ros::Rate loopRate(1000);
  while (ros::ok()) {

    auto availableBytes = serial.available();
    std::string strReceived;
    if (availableBytes) {
      serial.read(strReceived, availableBytes);
      frameExtraction.unpackData(strReceived);
    }
    loopRate.sleep();
  }
  return 1;
}
