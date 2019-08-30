#include "ncommon.h"

namespace LinkTrack {
const std::vector<std::string> kProductNames = {
    "I-UWB LPS S", "I-UWB LPS P",  "I-UWB LPS SA", "I-UWB LPS PA",
    "LinkTrack S", "LinkTrack SS", "LinkTrack M",  "LinkTrack MS",
    "LinkTrack P", "LinkTrack PS",
};
const std::vector<int> kBaudRates{100000,  115200,  230400,  460800,
                                  921600,  1000000, 1382400, 1843200,
                                  2000000, 2764800, 3000000};
}
