#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs(int fd, int speed, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		printf("error %d from tcgetattr", errno);
		exit(-1);
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
																	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
																	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("error %d from tcsetattr", errno);
    exit(-1);
	}
	return 0;
}

void set_blocking (int fd, int should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf("error %d from tggetattr", errno);
    exit(-1);
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("error %d setting term attributes", errno);
  }
}

char serial_data[8 * 16 * 4];
int open_serial_port(const char* portname) {
  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("error %d opening %s: %s", errno, portname, strerror (errno));
    exit(-1);
  }

  set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 0);                // set no blocking

  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 8; ++y) {
      serial_data[(x * 8 + y) * 4] = 255;
      serial_data[(x * 8 + y) * 4 + 1] = 255;
      serial_data[(x * 8 + y) * 4 + 2] = x * 8 + y;
      serial_data[(x * 8 + y) * 4 + 3] = 0;
    }
  }

  return fd;
}

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

int send_serial(int fd, int bars_count, int const f[200]) {
  /* printf("%d\n", bars_count); */
  /* for (int i = 0; i < bars_count; ++i) { */
  /*   printf("%d ", f[i]); */
  /* } */
  /* printf("\n"); */

  for (int y = 0; y < 64; ++y) {
    int v = f[y];
    if (v > 15) v = 15;
    if (v < 0) v = 0;
    int real_y = (int)(y / 8);
    unsigned char set = 1 << (7 - (y - real_y * 8));
    unsigned char unset = ~set;
    for (int x = 0; x < 16 - v; ++x) {
      serial_data[(x * 8 + real_y) * 4 + 3] &= unset;
    }
    for (int x = 16 - v; x < 16; ++x) {
      serial_data[(x * 8 + real_y) * 4 + 3] |= set;
    }
  }
  /* for (int x = 0; x < 16; ++x) { */
  /*   for (int y = 0; y < 8; ++y) { */
  /*     const unsigned char v = serial_data[(x * 8 + y) * 4 + 3]; */
  /*     printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(v)); */
  /*   } */
  /*   printf("\n"); */
  /* } */
  /* printf("\n"); */
  /* return 0; */


  write(fd, serial_data, sizeof(serial_data));
  return 0;
}
