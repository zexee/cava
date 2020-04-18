#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include "font8x8/font8x8_basic.h"

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

char serial_data[8 * 16 + 3 * 4];
int row_start[16];
// AA AA 00 4 * 8 BYTE
// AA AA 32 4 * 8 BYTE
// AA AA 64 4 * 8 BYTE
// AA AA 96 4 * 8 BYTE

void PrepareFont(char (*font)[8], int len) {
  for (int i = 0; i < len; ++i) {
    for (int j = 0; j < 8; ++j) {
      char x = font[i][j];
      char y = '\0';
      for (int k = 0; k < 8; ++k) {
        y |= ((x >> k) & 0x01) << (7 - k);
      }
      font[i][j] = ~y;
    }
  }
}
// Find a ttyUSB? in dev
int open_serial_port() {
  PrepareFont(font8x8_basic, sizeof(font8x8_basic) / sizeof(font8x8_basic[0]));
  DIR *d;
  struct dirent *dir;
  d = opendir("/dev");
  char portname[20] = "/dev/";
  if (d) {
    while ((dir = readdir(d)) != NULL) {
      if (strlen(dir->d_name) > 6 &&
          dir->d_name[0] == 't' &&
          dir->d_name[1] == 't' &&
          dir->d_name[2] == 'y' &&
          dir->d_name[3] == 'U' &&
          dir->d_name[4] == 'S' &&
          dir->d_name[5] == 'B') {
        strcat(portname, dir->d_name);
      }
    }
    closedir(d);
  }
  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("error %d opening %s: %s", errno, portname, strerror (errno));
    exit(-1);
  }

  set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 0);                // set no blocking

  memset(serial_data, 0, sizeof(serial_data));
  serial_data[0] = 0xAA;
  serial_data[1] = 0xAA;
  serial_data[2] = 0;
  row_start[0] = 3;
  row_start[1] = row_start[0] + 8;
  row_start[2] = row_start[1] + 8;
  row_start[3] = row_start[2] + 8;

  serial_data[3 + 4 * 8] = 0xAA;
  serial_data[3 + 4 * 8 + 1] = 0xAA;
  serial_data[3 + 4 * 8 + 2] = 32;
  row_start[4] = row_start[3] + 8 + 3;
  row_start[5] = row_start[4] + 8;
  row_start[6] = row_start[5] + 8;
  row_start[7] = row_start[6] + 8;

  serial_data[(3 + 4 * 8) * 2] = 0xAA;
  serial_data[(3 + 4 * 8) * 2 + 1] = 0xAA;
  serial_data[(3 + 4 * 8) * 2 + 2] = 64;
  row_start[8] = row_start[7] + 8 + 3;
  row_start[9] = row_start[8] + 8;
  row_start[10] = row_start[9] + 8;
  row_start[11] = row_start[10] + 8;

  serial_data[(3 + 4 * 8) * 3] = 0xAA;
  serial_data[(3 + 4 * 8) * 3 + 1] = 0xAA;
  serial_data[(3 + 4 * 8) * 3 + 2] = 96;
  row_start[12] = row_start[11] + 8 + 3;
  row_start[13] = row_start[12] + 8;
  row_start[14] = row_start[13] + 8;
  row_start[15] = row_start[14] + 8;
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

void Render(const char* str, int top_offset) {
  for (int i = 0; i < 8; ++i) {
    int x = str[i];
    char* c = font8x8_basic[x];
    for (int r = 0; r < 8; ++r) {
      serial_data[row_start[r + top_offset] + i] = c[r];
    }
  }
}

void RenderDateTime(const time_t* t) {
  struct tm* ti = localtime(t);
  char date[] = "06:23:5004/18 PM";
  if (ti->tm_hour == 12) {
    sprintf(date, "12:%02d:%02d%02d/%02d PM",
            ti->tm_min, ti->tm_sec,
            ti->tm_mon + 1, ti->tm_mday);
  } else if (ti->tm_hour > 12) {
    sprintf(date, "%02d:%02d:%02d%02d/%02d PM",
            ti->tm_hour - 12, ti->tm_min, ti->tm_sec,
            ti->tm_mon + 1, ti->tm_mday);
  } else {
    sprintf(date, "%02d:%02d:%02d%02d/%02d AM",
            ti->tm_hour, ti->tm_min, ti->tm_sec,
            ti->tm_mon + 1, ti->tm_mday);
  }
  Render(date, 0);
  Render(date + 8, 8);
}
int send_serial(int fd, int bars_count, int const f[200]) {
  time_t now = time(NULL);
  if (bars_count == 0) {
    RenderDateTime(&now);
  } else {
    for (int y = 0; y < 64; ++y) {
      int v = f[y];
      if (v > 16) v = 16;
      if (v < 0) v = 0;
      int real_y = (int)(y / 8);
      unsigned char set = 1 << (7 - (y - real_y * 8));
      unsigned char unset = ~set;
      for (int x = 0; x < 16 - v; ++x) {
        int pos = x * 8 + real_y;
        if (pos < 32) pos += 3;
        else if (pos < 64) pos += 6;
        else if (pos < 96) pos += 9;
        else pos += 12;
        serial_data[pos] |= set;
      }
      for (int x = 16 - v; x < 16; ++x) {
        int pos = x * 8 + real_y;
        if (pos < 32) pos += 3;
        else if (pos < 64) pos += 6;
        else if (pos < 96) pos += 9;
        else pos += 12;
        serial_data[pos] &= unset;
      }
    }
  }

  // for (int x = 0; x < 16; ++x) {
  //   for (int y = 0; y < 8; ++y) {
  //     int pos = x * 8 + y;
  //     if (pos < 32) pos += 3;
  //     else if (pos < 64) pos += 6;
  //     else if (pos < 96) pos += 9;
  //     else pos += 12;
  //     const unsigned char v = serial_data[pos];
  //     printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(v));
  //   }
  //   printf("\n");
  // }
  // printf("\n"); */
  // return 0;


  if (write(fd, serial_data, sizeof(serial_data)) < 0) exit(-1);
  return 0;
}

int send_serial2(int fd, int bars_count, int const f[200]) {
  time_t now = time(NULL);
  if (bars_count == 0) {
    RenderDateTime(&now);
  } else {
    // left
    for (int y = 0; y < 64; ++y) {
      int v = f[y];
      if (v > 8) v = 8;
      if (v < 0) v = 0;
      int yy = 63 - y;
      int real_y = (int)(yy / 8);
      unsigned char set = 1 << (7 - (yy - real_y * 8));
      unsigned char unset = ~set;
      for (int x = 0; x < 8 - v; ++x) {
        int pos = x * 8 + real_y;
        if (pos < 32) pos += 3;
        else if (pos < 64) pos += 6;
        else if (pos < 96) pos += 9;
        else pos += 12;
        serial_data[pos] |= set;
      }
      for (int x = 8 - v; x < 8; ++x) {
        int pos = x * 8 + real_y;
        if (pos < 32) pos += 3;
        else if (pos < 64) pos += 6;
        else if (pos < 96) pos += 9;
        else pos += 12;
        serial_data[pos] &= unset;
      }
    }
    // right
    for (int y = 0; y < 64; ++y) {
      int v = f[64 + y];
      if (v > 8) v = 8;
      if (v < 0) v = 0;
      int real_y = (int)(y / 8);
      unsigned char set = 1 << (7 - (y - real_y * 8));
      unsigned char unset = ~set;
      for (int x = 8; x < 8 + v; ++x) {
        int pos = x * 8 + real_y;
        if (pos < 32) pos += 3;
        else if (pos < 64) pos += 6;
        else if (pos < 96) pos += 9;
        else pos += 12;
        serial_data[pos] &= unset;
      }
      for (int x = 8 + v; x < 16; ++x) {
        int pos = x * 8 + real_y;
        if (pos < 32) pos += 3;
        else if (pos < 64) pos += 6;
        else if (pos < 96) pos += 9;
        else pos += 12;
        serial_data[pos] |= set;
      }
    }
  }

  /* for (int x = 0; x < 16; ++x) { */
  /*   for (int y = 0; y < 8; ++y) { */
  /*     int pos = x * 8 + y; */
  /*     if (pos < 32) pos += 3; */
  /*     else if (pos < 64) pos += 6; */
  /*     else if (pos < 96) pos += 9; */
  /*     else pos += 12; */
  /*     const unsigned char v = serial_data[pos]; */
  /*     printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(v)); */
  /*   } */
  /*   printf("\n"); */
  /* } */
  /* printf("\n"); */
  /* return 0; */

  if (write(fd, serial_data, sizeof(serial_data)) < 0) exit(-1);
  return 0;
}
