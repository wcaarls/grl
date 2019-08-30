// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

#include <grl/environments/lci.h>

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

using namespace grl;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

int Serial::open(std::string portname, int bps)
{
    fd_ = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        printf("Error opening %s: %s\n", portname.c_str(), strerror(errno));
        return -1;
    }
    
    // 8 bits, no parity, 1 stop bit
    set_interface_attribs(fd_, bps);
    
    // Pure timed read
    set_mincount(fd_, 0);
    
    return 0;
}

int Serial::read(unsigned char *buf, int sz)
{
  int n=0;
  
  do {
    int rdlen;
    rdlen = ::read(fd_, &buf[n], sz - n);
    if (rdlen > 0)
      n += rdlen;
    else if (rdlen < 0)
      printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    else
      printf("Timeout from read\n");
  } while (n < sz);
  
  return n;
}

void Serial::write(const unsigned char *buf, int sz)
{
  int wlen = ::write(fd_, buf, sz);
    
  if (wlen != sz)
    printf("Error from write: %d, %d\n", wlen, errno);
}

