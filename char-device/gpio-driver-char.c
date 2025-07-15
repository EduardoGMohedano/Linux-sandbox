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

#define DEVICE_NAME     "simplegpio"
#define CLASS_NAME      "gpio_class"
#define GPIO_COUNT      1  // Support up to 32 GPIOs

#define LED_GPIO_PIN 	17  // GPIO pin for LED (change as needed)
#define LED_GPIO_OFFSET 512  // GPIO pin for LED (change as needed)

// IOCTL commands
#define GPIO_MAGIC 'g'
#define GPIO_SET_OUTPUT     _IOW(GPIO_MAGIC, 0, int)
#define GPIO_SET_INPUT      _IOW(GPIO_MAGIC, 1, int)
#define GPIO_SET_HIGH       _IOW(GPIO_MAGIC, 2, int)
#define GPIO_SET_LOW        _IOW(GPIO_MAGIC, 3, int)
#define GPIO_GET_VALUE      _IOR(GPIO_MAGIC, 4, int)
#define GPIO_REQUEST_PIN    _IOW(GPIO_MAGIC, 5, int)
#define GPIO_FREE_PIN       _IOW(GPIO_MAGIC, 6, int)

// Device structure
struct gpio_device {
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    bool gpio_requested[GPIO_COUNT];
};

static struct gpio_device gpio_dev;
static struct gpio_desc *led_gpio = NULL; //Real struct to manage GPIO

// File operations prototypes
static int gpio_open(struct inode *inode, struct file *file);
static int gpio_release(struct inode *inode, struct file *file);
static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t gpio_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static ssize_t gpio_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset);

// File operations structure
static struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .open = gpio_open,
    .release = gpio_release,
    .unlocked_ioctl = gpio_ioctl,
    .read = gpio_read,
    .write = gpio_write,
};

static int gpio_open(struct inode *inode, struct file *file)
{
    pr_info("GPIO device opened\n");
    return 0;
}

static int gpio_release(struct inode *inode, struct file *file)
{
    pr_info("GPIO device closed\n");
    return 0;
}

static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int gpio_num;
    int ret = 0;
    int value;

    // Get GPIO number from user
    if (copy_from_user(&gpio_num, (int __user *)arg, sizeof(int))) {
        return -EFAULT;
    }

    // Validate GPIO number
    if (gpio_num < 0 || gpio_num >= GPIO_COUNT) {
        pr_err("Invalid GPIO number: %d\n", gpio_num);
        return -EINVAL;
    }

    switch (cmd) {
        case GPIO_REQUEST_PIN:
            if (gpio_dev.gpio_requested[gpio_num]) {
                pr_warn("GPIO %d already requested\n", gpio_num);
                return -EBUSY;
            }
            
            ret = gpio_request(gpio_num, "simplegpio");
            if (ret < 0) {
                pr_err("Failed to request GPIO %d: %d\n", gpio_num, ret);
                return ret;
            }
            
            gpio_dev.gpio_requested[gpio_num] = true;
            pr_info("GPIO %d requested successfully\n", gpio_num);
            break;

        case GPIO_FREE_PIN:
            if (!gpio_dev.gpio_requested[gpio_num]) {
                pr_warn("GPIO %d not requested\n", gpio_num);
                return -EINVAL;
            }
            
            gpiod_put(gpio_num);
            gpio_dev.gpio_requested[gpio_num] = false;
            pr_info("GPIO %d freed\n", gpio_num);
            break;

        case GPIO_SET_OUTPUT:
            if (!gpio_dev.gpio_requested[gpio_num]) {
                pr_err("GPIO %d not requested\n", gpio_num);
                return -EINVAL;
            }
            
            ret = gpio_direction_output(gpio_num, 0);
            if (ret < 0) {
                pr_err("Failed to set GPIO %d as output: %d\n", gpio_num, ret);
                return ret;
            }
            pr_info("GPIO %d set as output\n", gpio_num);
            break;

        case GPIO_SET_INPUT:
            if (!gpio_dev.gpio_requested[gpio_num]) {
                pr_err("GPIO %d not requested\n", gpio_num);
                return -EINVAL;
            }
            
            ret = gpio_direction_input(gpio_num);
            if (ret < 0) {
                pr_err("Failed to set GPIO %d as input: %d\n", gpio_num, ret);
                return ret;
            }
            pr_info("GPIO %d set as input\n", gpio_num);
            break;

        case GPIO_SET_HIGH:
            if (!gpio_dev.gpio_requested[gpio_num]) {
                pr_err("GPIO %d not requested\n", gpio_num);
                return -EINVAL;
            }
            
            gpio_set_value(gpio_num, 1);
            pr_info("GPIO %d set to HIGH\n", gpio_num);
            break;

        case GPIO_SET_LOW:
            if (!gpio_dev.gpio_requested[gpio_num]) {
                pr_err("GPIO %d not requested\n", gpio_num);
                return -EINVAL;
            }
            
            gpio_set_value(gpio_num, 0);
            pr_info("GPIO %d set to LOW\n", gpio_num);
            break;

        case GPIO_GET_VALUE:
            if (!gpio_dev.gpio_requested[gpio_num]) {
                pr_err("GPIO %d not requested\n", gpio_num);
                return -EINVAL;
            }
            
            value = gpio_get_value(gpio_num);
            if (copy_to_user((int __user *)arg, &value, sizeof(int))) {
                return -EFAULT;
            }
            pr_info("GPIO %d value: %d\n", gpio_num, value);
            break;

        default:
            pr_err("Invalid IOCTL command: %u\n", cmd);
            return -ENOTTY;
    }

    return ret;
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
        if (gpio_dev.gpio_requested[i]) {
            int value = gpio_get_value(i);
            int remaining = sizeof(msg) - msg_len;
            int added = snprintf(msg + msg_len, remaining, 
                               "GPIO %d: %s (value: %d)\n", 
                               i, 
                               gpiod_get_direction(i) ? "INPUT" : "OUTPUT", 
                               value);
            
            if (added >= remaining) {
                break; // Buffer full
            }
            msg_len += added;
            count++;
        }
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
    int gpio_num, value;
    int ret;

    if (len >= sizeof(cmd)) {
        return -EINVAL;
    }

    if (copy_from_user(cmd, buffer, len)) {
        return -EFAULT;
    }

    cmd[len] = '\0';

    sscanf(cmd, %d, &value);
    pr_info("You write %d to GPIO LED\n", value);
    gpiod_set_value(led_gpio, value);
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

static int __init gpio_driver_init(void)
{
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

    // First try the modern descriptor-based API
    led_gpio = gpio_to_desc( LED_GPIO_PIN + LED_GPIO_OFFSET );
    if (!led_gpio) {
           printk(KERN_INFO "GPIO driver: Error getting pin\n");
	   return -ENODEV;
    }
    
    // Set GPIO direction to output
    ret = gpiod_direction_output(led_gpio, 0);
    if (ret) {
        printk(KERN_ERR "GPIO driver: Failed to set GPIO direction\n");
        return ret;
    }
    
    gpiod_set_value(led_gpio, 0);

    return 0;

cleanup_class:
    class_destroy(gpio_dev.class);
cleanup_cdev:
    cdev_del(&gpio_dev.cdev);
cleanup_chrdev:
    unregister_chrdev_region(gpio_dev.dev_num, 1);
    return ret;
}

static void __exit gpio_driver_exit(void)
{
    int i;

    pr_info("Cleaning up GPIO driver\n");

    // Free all requested GPIOs
    for (i = 0; i < GPIO_COUNT; i++) {
        if (gpio_dev.gpio_requested[i]) {
            // gpiod_put(i);
            pr_info("Freed GPIO %d during cleanup\n", i);
        }
    }

    gpiod_put(led_gpio);

    // Clean up device
    device_destroy(gpio_dev.class, gpio_dev.dev_num);
    class_destroy(gpio_dev.class);
    cdev_del(&gpio_dev.cdev);
    unregister_chrdev_region(gpio_dev.dev_num, 1);

    pr_info("GPIO driver cleanup complete\n");
}

module_init(gpio_driver_init);
module_exit(gpio_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LALO");
MODULE_DESCRIPTION("GPIO control driver using /dev/ interface");
MODULE_VERSION("1.0");