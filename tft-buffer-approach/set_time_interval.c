#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

struct tft_color {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
};

#define TFT_IOC_MAGIC       'T'
#define TFT_SET_TIME_INTER  _IOW(TFT_IOC_MAGIC,5,int)

int main(int argc, char *argv[])
{
    int fd;
    int time = 0;
    
    if (argc != 2) {
        printf("Usage: %s <time_interval>\n", argv[0]);
        printf("Example: %s 100\n", argv[0]);
        return 1;
    }
    
    // Parse time values
    time = atoi(argv[1]);
    
    // // Validate range
    // if (color.red > 31 || color.green > 63 || color.blue > 31) {
    //     printf("Error: RGB values must respect RGB565 std, max 31 for RB and 63 for G\n");
    //     return 1;
    // }
    
    // Open device
    fd = open("/dev/simple-tft", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/simple-tft");
        return 1;
    }
    
    // Send ioctl command
    if (ioctl(fd, TFT_SET_TIME_INTER, &time) < 0) {
        perror("ioctl failed");
        close(fd);
        return 1;
    }
    
    printf("Time interval in millseconds set to (%d)\n", time);
    
    close(fd);
    return 0;
}