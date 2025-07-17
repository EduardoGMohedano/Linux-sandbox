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

#define DEVICE_NAME     "simple-tft"
#define CLASS_NAME      "tft_class"

// GPIO pins for SPI communication
#define TFT_PIN_RST 	17  
#define TFT_PIN_OFFSET  512  // Offset for gpiochip1


// File operations prototypes
static int gpio_open(struct inode *inode, struct file *file);
static int gpio_release(struct inode *inode, struct file *file);
// static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t gpio_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static ssize_t gpio_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset);

// File operations structure
static struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .open = gpio_open,
    .release = gpio_release,
    // .unlocked_ioctl = gpio_ioctl,
    .read = gpio_read,
    .write = gpio_write,
};

static int gpio_open(struct inode *inode, struct file *file){
    pr_info("TFT device opened\n");
    return 0;
}

static int gpio_release(struct inode *inode, struct file *file){
    pr_info("TFT device closed\n");
    return 0;
}

static ssize_t gpio_read(struct file *file, char __user *buffer, size_t len, loff_t *offset)
{
    char msg[256];
    int msg_len;
    int i, count = 0;

    if (*offset > 0) {
        return 0; // EOF
    }

    // Show status of all requested GPIOs
    msg_len = snprintf(msg, sizeof(msg), "GPIO Status:\n");
    
    for (i = 0; i < GPIO_COUNT; i++) {
        int value = gpiod_get_value(gpio_dev.led_gpio[i]);
        int remaining = sizeof(msg) - msg_len;
        int added = snprintf(msg + msg_len, remaining, 
                            "GPIO %d: %s (value: %d)\n", 
                            i, 
                            gpiod_get_direction(gpio_dev.led_gpio[i]) ? "INPUT" : "OUTPUT", 
                            value);
        
        if (added >= remaining) {
            break; // Buffer full
        }
        msg_len += added;
        count++;
    }

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

static ssize_t gpio_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset)
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
    for(int i = 0; i < GPIO_COUNT; i++)
        gpiod_set_value(gpio_dev.led_gpio[i], (value >> i) & 1 );
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

static int __init tft_driver_init(void)
{

    return spi_register_driver(&tft_driver);


    int ret;

    pr_info("Initializing GPIO driver\n");

    // Initialize gpio_requested array
    memset(gpio_dev.gpio_requested, false, sizeof(gpio_dev.gpio_requested));

    // Allocate device number (device, minor_number, number of devoces, name)
    ret = alloc_chrdev_region(&gpio_dev.dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate device number: %d\n", ret);
        return ret;
    }

    pr_info("Device number allocated: Major=%d, Minor=%d\n", MAJOR(gpio_dev.dev_num), MINOR(gpio_dev.dev_num));

    // Initialize character device
    cdev_init(&gpio_dev.cdev, &gpio_fops);
    gpio_dev.cdev.owner = THIS_MODULE;

    // Add character device to system
    ret = cdev_add(&gpio_dev.cdev, gpio_dev.dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to add character device: %d\n", ret);
        goto cleanup_chrdev;
    }

    // Create device class
    gpio_dev.class = class_create(CLASS_NAME);
    if (IS_ERR(gpio_dev.class)) {
        ret = PTR_ERR(gpio_dev.class);
        pr_err("Failed to create device class: %d\n", ret);
        goto cleanup_cdev;
    }

    // Create device file
    gpio_dev.device = device_create(gpio_dev.class, NULL, gpio_dev.dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(gpio_dev.device)) {
        ret = PTR_ERR(gpio_dev.device);
        pr_err("Failed to create device: %d\n", ret);
        goto cleanup_class;
    }

    pr_info("GPIO driver initialized successfully. Device: /dev/%s\n", DEVICE_NAME);

    // Using modern descriptor-based API
    for(int i = 0; i < GPIO_COUNT; i++){
        gpio_dev.led_gpio[i] = gpio_to_desc(gpio_array[i]);
        if (!gpio_dev.led_gpio[i]){
            printk(KERN_INFO "GPIO driver: Error getting pin\n");
            return -ENODEV;
        }
        
        // Set GPIO direction to output
        ret = gpiod_direction_output(gpio_dev.led_gpio[i], 0);
        if (ret) {
            printk(KERN_ERR "GPIO driver: Failed to set GPIO direction\n");
            return ret;
        }
        
        gpiod_set_value(gpio_dev.led_gpio[i], 0);
    }

    return 0;

cleanup_class:
    class_destroy(gpio_dev.class);
cleanup_cdev:
    cdev_del(&gpio_dev.cdev);
cleanup_chrdev:
    unregister_chrdev_region(gpio_dev.dev_num, 1);
    return ret;
}

static void __exit tft_driver_exit(void){
    
    spi_unregister_driver(&tft_driver);

    pr_info("Cleaning up GPIO driver\n");

    // Free all requested GPIOs
    for (int i = 0; i < GPIO_COUNT; i++) {
        gpiod_set_value(gpio_dev.led_gpio[i], 0);
        gpiod_put(gpio_dev.led_gpio[i]);
    	pr_info("Freed GPIO %d during cleanup\n", gpio_array[i]);
    }

    // Clean up device
    device_destroy(gpio_dev.class, gpio_dev.dev_num);
    class_destroy(gpio_dev.class);
    cdev_del(&gpio_dev.cdev);
    unregister_chrdev_region(gpio_dev.dev_num, 1);

    pr_info("GPIO driver cleanup complete\n");
}


// In probe function
static int tft_probe(struct spi_device *spi){
    spi->mode = SPI_MODE_0; // or appropriate mode for your TFT
    spi->bits_per_word = 8;
    spi->max_speed_hz = 10000000; // 32MHz or your TFT's max speed
    
    return spi_setup(spi);
}

static void tft_remove(struct spi_device *spi){
    
}


static const struct of_device_id tft_of_match[] = {
    { .compatible = "raio,tft-display" },
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
