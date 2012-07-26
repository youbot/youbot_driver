/////////////////////////////////////////////////
// Serial port interface program to write the IP-Address on the youBot display
/////////////////////////////////////////////////

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <iostream>
#include <string>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <cstdlib>

enum displayline {
  line2 = 0x02,
  line3 = 0x03
};

enum voltagesource {
  battery1 = 0x04,
  battery2 = 0x05,
  powersupply = 0x0c
};

int open_port(std::string port) {
  int fd; // file description for the serial port

  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1) // if open is unsucessful
  {
    throw ("unable to open port: " + port);
  } else {
    fcntl(fd, F_SETFL, 0);
    printf("port is open.\n");
  }

  return (fd);
}

int configure_port(int fd) // configure the port
{
  struct termios port_settings; // structure to store the port settings in

  tcgetattr(fd, &port_settings);

  cfsetispeed(&port_settings, B0); // set baud rates
  cfsetospeed(&port_settings, B0);


  port_settings.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode...

  //port_settings.c_cflag |= CRTSCTS; //Enable Hardware Flow Control
  port_settings.c_cflag &= ~CRTSCTS; //Disable Hardware Flow Control

  port_settings.c_cflag &= ~PARENB; // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Choosing Raw Input
  port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software flow control
  port_settings.c_oflag &= ~OPOST; //Choosing Raw Output
  port_settings.c_cc[VMIN] = 0;
  port_settings.c_cc[VTIME] = 10; /* set raw input, 1 second timeout */

  tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port
  return (fd);

}

bool setText(int fd, displayline line, std::string text) {
  const int size = 18;
  if (text.size() > size - 2) {
    printf("The text is to long!\n");
    return false;
  }

  unsigned char send_bytes[size];

  send_bytes[0] = line;
  for (unsigned int i = 0; i < size - 2; i++) {
    if (i < text.size())
      send_bytes[i + 1] = text[i];
    else
      send_bytes[i + 1] = ' ';
  }
  send_bytes[size - 1] = 0x0D; //trailing \CR

  write(fd, send_bytes, size); //Send data
  printf("[");
  for (int i = 1; i < size - 1; i++) {
    printf("%c", send_bytes[i]);
  }
  printf("]\n");

  return true;
}

//return the voltage in [Volt]
double getVoltage(int fd, voltagesource source) {
  unsigned char send_bytes[1];
  send_bytes[0] = source;

  write(fd, send_bytes, 1); //Send data

  const int readsize = 20; 
  char read_bytes[readsize] = {0};

  int nobytesread = 0;
  nobytesread = read(fd, read_bytes, readsize);
  read_bytes[nobytesread-1] = 0;  //delete the last tow character \CR+\LF
  read_bytes[nobytesread-2] = 0;

  std::string value(read_bytes);

  return (double)atoi(value.c_str())/1000;
}

void getIPAdress(std::string lanname, std::string wlanname, std::string& lanip, std::string& wlanip) {
  struct ifaddrs * ifAddrStruct = NULL;
  struct ifaddrs * ifa = NULL;
  void * tmpAddrPtr = NULL;
  lanip = "L";
  wlanip = "W";
  getifaddrs(&ifAddrStruct);

  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa ->ifa_addr->sa_family == AF_INET) { // check it is IP4
      // is a valid IP4 Address
      tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      //  printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
      std::string interface(ifa->ifa_name);
      if (interface == lanname)
        lanip.append(addressBuffer);
      if (interface == wlanname)
        wlanip.append(addressBuffer);
    }
  }
  if (ifAddrStruct != NULL) freeifaddrs(ifAddrStruct);

  return;
}

int main(int argc, char* argv[]) {
  try {
    if (argc != 4) {
      std::cout << "invalid arguments \n ./displayipaddress 'serial port' 'ethernet' 'wlan' \n./displayipaddress /dev/ttyACM0 eth1 wlan0" << std::endl;
      return 0;
    }
    int fd = open_port(argv[1]);
    configure_port(fd);

    std::string lanip;
    std::string wlanip;

    while (true) {
      sleep(2);
      getIPAdress(argv[2], argv[3], lanip, wlanip);
      setText(fd, line2, lanip);
      setText(fd, line3, wlanip);
   //   std::cout << "Bat1: " << getVoltage(fd, battery1) << std::endl;
   //   std::cout << "Bat2: " << getVoltage(fd, battery2) << std::endl;
   //   std::cout << "powersupply: " << getVoltage(fd, powersupply) << std::endl;
    }

    close(fd);
  } catch (std::string &e) {
    std::cout << "Error: " << e << std::endl;
  }
  return (0);
}

