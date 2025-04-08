#include <ncurses.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int main(int argc, char *argv[])
{
    int ch = 2, c;
    initscr();
    cbreak();
    noecho();

    int fd;

    if ((fd = serialOpen("/dev/serial0", 38400)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    if (wiringPiSetup() == -1)
    {
        fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
        return 1;
    }

    serialFlush(fd);
    timeout(50);

    while (true)
    {
        char c = getch();

        switch (c)
        {
        case 'w': // forward
            ch = 1;
            break;
        case 'a': // left
            ch = 3;
            break;
        case 's': // reverse
            ch = 5;
            break;
        case 'd': // right
            ch = 4;
            break;
        case 'q': // claw open
            ch = 6;
            break;
        case 'e': // claw close
            ch = 7;
            break;
        case 'x': // medpack release
            ch = 0;
            break;
        case 'o':
            endwin();
            return 0;
            break;
        default: // stop
            ch = 2;
            break;
        }

        serialPutchar(fd, (uint8_t)ch);

        delay(20);

        if (serialDataAvail(fd))
        {
            char received = serialGetchar(fd);
            if (received == 0)
            {
                printw("Red  \r");
            }
            else if (received == 1)
            {
                printw("Green\r");
            }
            else
            {
                printw("Black\r");
            }
        }
    }

    endwin();
    return 0;
}
