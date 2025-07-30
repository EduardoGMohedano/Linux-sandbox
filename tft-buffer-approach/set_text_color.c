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

#define TFT_IOC_MAGIC 'T'
#define TFT_SET_TEXT_COLOR _IOW(TFT_IOC_MAGIC, 1, struct tft_color)

int main(int argc, char *argv[])
{
    int fd;
    struct tft_color color;
    
    if (argc != 4) {
        printf("Usage: %s <red> <green> <blue>\n", argv[0]);
        printf("Example: %s 255 0 0\n", argv[0]);
        return 1;
    }
    
    // Parse RGB values
    color.red = atoi(argv[1]);
    color.green = atoi(argv[2]);
    color.blue = atoi(argv[3]);
    
    // Validate range
    if (color.red > 31 || color.green > 63 || color.blue > 31) {
        printf("Error: RGB values must respect RGB565 std, max 31 for RB and 63 for G\n");
        return 1;
    }
    
    // Open device
    fd = open("/dev/simple-tft", O_RDWR);
    if (fd < 0) {
        perror("Failed to open /dev/simple-tft");
        return 1;
    }
    
    // Send ioctl command
    if (ioctl(fd, TFT_SET_TEXT_COLOR, &color) < 0) {
        perror("ioctl failed");
        close(fd);
        return 1;
    }
    
    printf("Text color set to RGB(%d, %d, %d)\n", color.red, color.green, color.blue);
    
    close(fd);
    return 0;
}