#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DEVICE_NAME     "gpio_blink"
#define CLASS_NAME     "gpio_blink_class"
#define LED_GPIO_PIN 	17  // GPIO pin for LED (change as needed)
#define LED_GPIO_OFFSET 512  // GPIO pin for LED (change as needed)
#define BLINK_INTERVAL 	1000  // Blink interval in milliseconds

static struct task_struct *blink_thread;
static int led_state = 0;
static int stop_thread = 0;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LALO");
MODULE_DESCRIPTION("Simple LED Blink Module for Raspberry Pi");
MODULE_VERSION("1.0");

static struct gpio_desc *led_gpio = NULL;

struct gpio_blink_dev {
    struct cdev cdev;
    struct device *device;
    struct gpio_desc *gpio_desc;
    struct timer_list blink_timer;
    int blink_interval_ms;
    bool blink_enabled;
    bool gpio_state;
    struct mutex lock;
};

static dev_t dev_num;
static struct class *gpio_blink_class;
static struct gpio_blink_dev *gpio_blink_device;

// Timer callback function
static void blink_timer_callback(struct timer_list *t)
{
    struct gpio_blink_dev *dev = from_timer(dev, t, blink_timer);
    
    mutex_lock(&dev->lock);
    
    if (dev->blink_enabled) {
        // Toggle GPIO state
        dev->gpio_state = !dev->gpio_state;
        gpiod_set_value(dev->gpio_desc, dev->gpio_state);
        
        // Restart timer
        mod_timer(&dev->blink_timer, 
                  jiffies + msecs_to_jiffies(dev->blink_interval_ms));
    }
    
    mutex_unlock(&dev->lock);
}


// Thread function that blinks the LED
static int led_blink_func(void *data)
{
    printk(KERN_INFO "LED blink thread started\n");
    
    while (!kthread_should_stop() && !stop_thread) {
        // Toggle LED state
        led_state = !led_state;
        gpiod_set_value(led_gpio, led_state);
        
        printk(KERN_INFO "LED state: %s\n", led_state ? "ON" : "OFF");
        
        // Sleep for the specified interval
        msleep(BLINK_INTERVAL);
    }
    
    // Turn off LED when thread exits
    gpiod_set_value(led_gpio, 0);
    printk(KERN_INFO "LED blink thread stopped\n");
    
    return 0;
}

// File operations: open
static int gpio_blink_open(struct inode *inode, struct file *file){
    struct gpio_blink_dev *dev;
    
    dev = container_of(inode->i_cdev, struct gpio_blink_dev, cdev);
    file->private_data = dev;
    
    pr_info("GPIO Blink device opened\n");
    return 0;
}

// File operations: release
static int gpio_blink_release(struct inode *inode, struct file *file){
    pr_info("GPIO Blink device closed\n");
    return 0;
}

// File operations: read
static ssize_t gpio_blink_read(struct file *file, char __user *buf, size_t count, loff_t *ppos){
    struct gpio_blink_dev *dev = file->private_data;
    char status[64];
    int len;
    
    mutex_lock(&dev->lock);
    
    len = snprintf(status, sizeof(status), 
                   "blink_enabled: %s\ninterval_ms: %d\ngpio_state: %s\n",
                   dev->blink_enabled ? "true" : "false",
                   dev->blink_interval_ms,
                   dev->gpio_state ? "high" : "low");
    
    mutex_unlock(&dev->lock);
    
    if (*ppos >= len)
        return 0;
    
    if (count > len - *ppos)
        count = len - *ppos;
    
    if (copy_to_user(buf, status + *ppos, count))
        return -EFAULT;
    
    *ppos += count;
    return count;
}

// File operations: write
static ssize_t gpio_blink_write(struct file *file, const char __user *buf,size_t count, loff_t *ppos){
    struct gpio_blink_dev *dev = file->private_data;
    char *kbuf;
    char *cmd, *value;
    int ret = count;
    
    if (count > 64)
        return -EINVAL;
    
    kbuf = kzalloc(count + 1, GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;
    
    if (copy_from_user(kbuf, buf, count)) {
        ret = -EFAULT;
        goto out;
    }
    
    // Remove trailing newline
    if (kbuf[count - 1] == '\n')
        kbuf[count - 1] = '\0';
    
    mutex_lock(&dev->lock);
    
    // Parse command
    cmd = strsep(&kbuf, " ");
    value = kbuf;
    
    if (strcmp(cmd, "start") == 0) {
        if (!dev->blink_enabled) {
            dev->blink_enabled = true;
            mod_timer(&dev->blink_timer, 
                      jiffies + msecs_to_jiffies(dev->blink_interval_ms));
            pr_info("GPIO blinking started\n");
        }
    } else if (strcmp(cmd, "stop") == 0) {
        if (dev->blink_enabled) {
            dev->blink_enabled = false;
            del_timer_sync(&dev->blink_timer);
            pr_info("GPIO blinking stopped\n");
        }
    } else if (strcmp(cmd, "interval") == 0 && value) {
        int new_interval;
        if (kstrtoint(value, 10, &new_interval) == 0 && new_interval > 0) {
            dev->blink_interval_ms = new_interval;
            pr_info("Blink interval set to %d ms\n", new_interval);
        } else {
            ret = -EINVAL;
        }
    } else if (strcmp(cmd, "set") == 0 && value) {
        int gpio_val;
        if (kstrtoint(value, 10, &gpio_val) == 0) {
            // Stop blinking and set manual state
            dev->blink_enabled = false;
            del_timer_sync(&dev->blink_timer);
            dev->gpio_state = gpio_val ? true : false;
            gpiod_set_value(dev->gpio_desc, dev->gpio_state);
            pr_info("GPIO manually set to %s\n", 
                    dev->gpio_state ? "high" : "low");
        } else {
            ret = -EINVAL;
        }
    } else {
        ret = -EINVAL;
    }
    
    mutex_unlock(&dev->lock);
    
out:
    kfree(kbuf);
    return ret;
}

static const struct file_operations gpio_blink_fops = {
    .owner = THIS_MODULE,
    .open = gpio_blink_open,
    .release = gpio_blink_release,
    .read = gpio_blink_read,
    .write = gpio_blink_write,
};

// Platform driver probe function
static int gpio_blink_probe(struct platform_device *pdev){
    struct device *dev = &pdev->dev;
    int ret;
    
    // Allocate device structure
    gpio_blink_device = devm_kzalloc(dev, sizeof(*gpio_blink_device), GFP_KERNEL);
    if (!gpio_blink_device){
        printk(KERN_ALERT "Failed to allocate device struct using devm_kzcalloc()");
        return -ENOMEM;
    }
    
    // Get GPIO descriptor from device tree
    gpio_blink_device->gpio_desc = devm_gpiod_get(dev, "blink", GPIOD_OUT_LOW); //TODO CHECK THE PROPER NAME OF THIS, IN CASE BAD NAME I WILL GET AN ERROR
    if (IS_ERR(gpio_blink_device->gpio_desc)) {
        ret = PTR_ERR(gpio_blink_device->gpio_desc);
        dev_err(dev, "Failed to get GPIO descriptor: %d\n", ret);
        return ret;
    }
    
    // Initialize device structure
    gpio_blink_device->blink_interval_ms = BLINK_INTERVAL;
    gpio_blink_device->blink_enabled = false;
    gpio_blink_device->gpio_state = false;
    mutex_init(&gpio_blink_device->lock);
    
    // Initialize timer
    timer_setup(&gpio_blink_device->blink_timer, blink_timer_callback, 0);
    
    // Allocate character device number
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(dev, "Failed to allocate character device number: %d\n", ret);
        return ret;
    }
    
    // Initialize and add character device
    cdev_init(&gpio_blink_device->cdev, &gpio_blink_fops);
    gpio_blink_device->cdev.owner = THIS_MODULE;
    
    ret = cdev_add(&gpio_blink_device->cdev, dev_num, 1);
    if (ret < 0) {
        dev_err(dev, "Failed to add character device: %d\n", ret);
        goto err_unregister_chrdev;
    }
    
    // Create device class
    gpio_blink_class = class_create(CLASS_NAME);
    if (IS_ERR(gpio_blink_class)) {
        ret = PTR_ERR(gpio_blink_class);
        dev_err(dev, "Failed to create device class: %d\n", ret);
        goto err_cdev_del;
    }
    
    // Create device file
    gpio_blink_device->device = device_create(gpio_blink_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(gpio_blink_device->device)) {
        ret = PTR_ERR(gpio_blink_device->device);
        dev_err(dev, "Failed to create device: %d\n", ret);
        goto err_class_destroy;
    }
    
    dev_info(dev, "GPIO Blink driver probed successfully\n");
    dev_info(dev, "Device created: /dev/%s\n", DEVICE_NAME);
    
    return 0;
    
err_class_destroy:
    class_destroy(gpio_blink_class);
err_cdev_del:
    cdev_del(&gpio_blink_device->cdev);
err_unregister_chrdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

// Platform driver remove function
static void gpio_blink_remove(struct platform_device *pdev)
{
    // Stop timer
    del_timer_sync(&gpio_blink_device->blink_timer);
    
    // Clean up character device
    device_destroy(gpio_blink_class, dev_num);
    class_destroy(gpio_blink_class);
    cdev_del(&gpio_blink_device->cdev);
    unregister_chrdev_region(dev_num, 1);
    
    dev_info(&pdev->dev, "GPIO Blink driver removed\n");
    // return 0;
}

// Device tree match table
static const struct of_device_id gpio_blink_of_match[] = {
    { .compatible = "custom,gpio-blink" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gpio_blink_of_match);

// Platform driver structure
static struct platform_driver gpio_blink_driver = {
    .probe = gpio_blink_probe,
    .remove = gpio_blink_remove,
    .driver = {
        .name = "gpio-blink",
        .of_match_table = gpio_blink_of_match,
    },
};

// Module initialization
static int __init gpio_blink_init(void)
{
    pr_info("GPIO Blink driver loading\n");
    return platform_driver_register(&gpio_blink_driver);
}

// Module cleanup
static void __exit gpio_blink_exit(void)
{
    platform_driver_unregister(&gpio_blink_driver);
    pr_info("GPIO Blink driver unloaded\n");
}

module_init(gpio_blink_init);
module_exit(gpio_blink_exit);

// // Module initialization function
// static int __init led_blink_init(void){
//     int ret;
    
//     printk(KERN_INFO "LED Blink: Initializing module\n");
    
//     // First try the modern descriptor-based API
//     led_gpio = gpio_to_desc( LED_GPIO_PIN + LED_GPIO_OFFSET);
//     if (!led_gpio) {
//            printk(KERN_INFO "LED Blink: Error getting pin\n");
// 	   return -ENODEV;
//     }
    
//     // Set GPIO direction to output
//     ret = gpiod_direction_output(led_gpio, 0);
//     if (ret) {
//         printk(KERN_ERR "LED Blink: Failed to set GPIO direction\n");
//         return ret;
//     }
    
//     gpiod_set_value(led_gpio, 1); 

//     // Create and start the kernel thread
//     blink_thread = kthread_run(led_blink_func, NULL, "led_blink_thread");
//     if (IS_ERR(blink_thread)) {
//         printk(KERN_ERR "Failed to create kernel thread\n");
//         gpiod_put(led_gpio);
//         return PTR_ERR(blink_thread);
//     }
    
//     return 0;
// }

// // Module cleanup function
// static void __exit led_blink_exit(void){
//     // Turn off LED and free GPIO
//     //
//     stop_thread = 1;

//     // Stop the kernel thread
//     if (blink_thread) {
//         kthread_stop(blink_thread);
//         blink_thread = NULL;
//     }

//     gpiod_set_value(led_gpio, 0);
//     gpiod_put(led_gpio);
//     printk(KERN_INFO "LED Blink: Module unloaded successfully\n");
// }

// // Register module entry and exit points
// module_init(led_blink_init);
// module_exit(led_blink_exit);
