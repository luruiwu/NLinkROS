#include "initserial.h"

#include <cstdio>
#include <iostream>
#include <string>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void mSleep(unsigned long milliseconds) {
#ifdef _WIN32
  Sleep(milliseconds); // 100 ms
#else
  usleep(milliseconds * 1000); // 100 ms
#endif
}

void enumerate_ports() {
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
  std::string test;
  test.clear();
}

void print_usage() {
  cerr << "Usage: test_serial {-e|<serial port address>} ";
  cerr << "<baudrate> [test string]" << endl;
}

void initSerial(int argc, char **argv, serial::Serial *serial) {
  try {
    if (argc < 2) {
      print_usage();
      exit(0);
    }
    // Argument 1 is the serial port or enumerate flag
    string port(argv[1]);

    if (port == "-e") {
      enumerate_ports();
      exit(0);
    } else if (argc < 3) {
      print_usage();
      exit(0);
    }

    // Argument 2 is the baudrate
    unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
    sscanf_s(argv[2], "%lu", &baud);
#else
    sscanf(argv[2], "%lu", &baud);
#endif

    // port, baudrate, timeout in milliseconds
    serial->setPort(port);
    serial->setBaudrate(baud);
    //  serial->setTimeout(serial::Timeout::simpleTimeout(1000));
    serial->open();
    //  serial = new serial::Serial(port, baud,
    //  serial::Timeout::simpleTimeout(1000));

    cout << "Is the serial port open?";
    if (serial->isOpen()) {
      cout << " Yes. Start working" << endl;
    } else {
      cout << " No. Please check and retry." << endl;
      exit(0);
    }
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
    exit(0);
  }
}
