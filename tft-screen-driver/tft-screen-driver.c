#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>

#define DEVICE_NAME     "simple-tft"
#define CLASS_NAME      "tft_class"

// GPIO pins for SPI communication
#define TFT_PIN_RST 	17  
#define TFT_PIN_OFFSET  512  // Offset for gpiochip1

dev_t dev_num;
struct cdev cdev;
struct class *class;
struct device *device;
struct gpio_desc *rst_gpio; //Real struct to manage GPIO

// File operations prototypes
static int tft_open(struct inode *inode, struct file *file);
static int tft_release(struct inode *inode, struct file *file);
// static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t tft_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static ssize_t tft_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset);

// File operations structure
static struct file_operations tft_fops = {
    .owner = THIS_MODULE,
    .open = tft_open,
    .release = tft_release,
    .read = tft_read,
    .write = tft_write,
};

static int tft_open(struct inode *inode, struct file *file){
    pr_info("TFT device opened\n");
    return 0;
}

static int tft_release(struct inode *inode, struct file *file){
    pr_info("TFT device closed\n");
    return 0;
}

static ssize_t tft_read(struct file *file, char __user *buffer, size_t len, loff_t *offset)
{
    char msg[256];
    int msg_len;
    int i, count = 0;

    if (*offset > 0) {
        return 0; // EOF
    }

    // Show status of all requested GPIOs
    msg_len = snprintf(msg, sizeof(msg), "GPIO Status:\n");
    
    // for (i = 0; i < GPIO_COUNT; i++) {
    //     int value = gpiod_get_value(gpio_dev.led_gpio[i]);
    //     int remaining = sizeof(msg) - msg_len;
    //     int added = snprintf(msg + msg_len, remaining, 
    //                         "GPIO %d: %s (value: %d)\n", 
    //                         i, 
    //                         gpiod_get_direction(gpio_dev.led_gpio[i]) ? "INPUT" : "OUTPUT", 
    //                         value);
        
    //     if (added >= remaining) {
    //         break; // Buffer full
    //     }
    //     msg_len += added;
    //     count++;
    // }

    if (count == 0) {
        msg_len = snprintf(msg, sizeof(msg), "No GPIOs requested\n");
    }

    if (len < msg_len) {
        return -EINVAL;
    }

    if (copy_to_user(buffer, msg, msg_len)) {
        return -EFAULT;
    }

    *offset = msg_len;
    return msg_len;
}

static ssize_t tft_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset)
{
    char cmd[64];
    int value;

    if (len >= sizeof(cmd)) {
        return -EINVAL;
    }

    if (copy_from_user(cmd, buffer, len)) {
        return -EFAULT;
    }

    cmd[len] = '\0';

    sscanf(cmd, "%d", &value);
    pr_info("You write %d to GPIO LED\n", value);
    // for(int i = 0; i < GPIO_COUNT; i++)
    //     gpiod_set_value(gpio_dev.led_gpio[i], (value >> i) & 1 );
    return len;

    // Simple command parser: "gpio_num value" (e.g., "18 1" sets GPIO 18 to HIGH)
    // if (sscanf(cmd, "%d %d", &gpio_num, &value) == 2) {
    //     if (gpio_num < 0 || gpio_num >= GPIO_COUNT) {
    //         pr_err("Invalid GPIO number: %d\n", gpio_num);
    //         return -EINVAL;
    //     }

    //     if (!gpio_dev.gpio_requested[gpio_num]) {
    //         pr_err("GPIO %d not requested\n", gpio_num);
    //         return -EINVAL;
    //     }

    //     gpio_set_value(gpio_num, value ? 1 : 0);
    //     pr_info("GPIO %d set to %s via write\n", gpio_num, value ? "HIGH" : "LOW");
    //     return len;
    // }

    // pr_err("Invalid write format. Use: 'gpio_num value'\n");
    // return -EINVAL;
}

static int tft_driver_i(void){
    int ret;

    pr_info("Initializing TFT driver\n");

    // Allocate device number (device, minor_number, number of devoces, name)
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate device number: %d\n", ret);
        return ret;
    }

    pr_info("Device number allocated: Major=%d, Minor=%d\n", MAJOR(dev_num), MINOR(dev_num));

    // Initialize character device
    cdev_init(&cdev, &tft_fops);
    cdev.owner = THIS_MODULE;

    // Add character device to system
    ret = cdev_add(&cdev, dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to add character device: %d\n", ret);
        goto cleanup_chrdev;
    }

    // Create device class
    class = class_create(CLASS_NAME);
    if (IS_ERR(class)) {
        ret = PTR_ERR(class);
        pr_err("Failed to create device class: %d\n", ret);
        goto cleanup_cdev;
    }

    // Create device file
    device = device_create(class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(device)) {
        ret = PTR_ERR(device);
        pr_err("Failed to create device: %d\n", ret);
        goto cleanup_class;
    }

    pr_info("TFT driver initialized successfully. Device: /dev/%s\n", DEVICE_NAME);

    // Using modern descriptor-based API for RESET PIN
    
    rst_gpio = gpio_to_desc(TFT_PIN_RST+TFT_PIN_OFFSET);
    if (!rst_gpio){
        printk(KERN_INFO "GPIO driver: Error getting RST pin\n");
        return -ENODEV;
    }
    
    // Set GPIO direction to output
    ret = gpiod_direction_output(rst_gpio, 0);
    if (ret) {
        printk(KERN_ERR "GPIO driver: Failed to set GPIO direction\n");
        return ret;
    }
    
    gpiod_set_value(rst_gpio, 0);

    return 0;

cleanup_class:
    class_destroy(class);
cleanup_cdev:
    cdev_del(&cdev);
cleanup_chrdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void tft_driver_e(void){
    
    pr_info("Cleaning up TFT driver\n");

    // Free all requested GPIOs
    gpiod_set_value(rst_gpio, 0);
    gpiod_put(rst_gpio);
    pr_info("Freed RST pin during cleanup\n");

    // Clean up device
    device_destroy(class, dev_num);
    class_destroy(class);
    cdev_del(&cdev);
    unregister_chrdev_region(dev_num, 1);

    pr_info("TFT driver cleanup complete\n");
}


// In probe function
static int tft_probe(struct spi_device *spi){
    spi->mode = SPI_MODE_3; // or appropriate mode for your TFT
    spi->bits_per_word = 8;
    spi->max_speed_hz = 10000000; // 32MHz or your TFT's max speed

    tft_driver_i();
    
    // return spi_setup(spi);
    return 0;
}

static void tft_remove(struct spi_device *spi){
    tft_driver_e(); 
}


static const struct of_device_id tft_of_match[] = {
    { .compatible = "custom,tft-screen" },
    { }
};

MODULE_DEVICE_TABLE(of, tft_of_match);

static struct spi_driver tft_driver = {
    .driver = {
        .name = "tft-display",
        .of_match_table = tft_of_match,
    },
    .probe = tft_probe,
    .remove = tft_remove,
};

module_spi_driver(tft_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("LALO");
MODULE_DESCRIPTION("TFT screen driver using /dev/ interface for RA8875");
MODULE_VERSION("1.0");
