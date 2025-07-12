#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "gpio_test_driver"
#define PROC_ENTRY_NAME "gpio_test"

static struct gpio_desc *test_gpio;
static struct proc_dir_entry *proc_entry;

// Proc file operations
static ssize_t gpio_proc_write(struct file *file, const char __user *buffer, 
                              size_t count, loff_t *pos)
{
    char value;
    
    if (count < 1)
        return -EINVAL;
    
    if (copy_from_user(&value, buffer, 1))
        return -EFAULT;
    
    if (value == '1') {
        gpiod_set_value(test_gpio, 1);
        pr_info("GPIO set to HIGH\n");
    } else if (value == '0') {
        gpiod_set_value(test_gpio, 0);
        pr_info("GPIO set to LOW\n");
    }
    
    return count;
}

static ssize_t gpio_proc_read(struct file *file, char __user *buffer, 
                             size_t count, loff_t *pos)
{
    char value_str[8];
    int len;
    int gpio_value;
    
    if (*pos > 0)
        return 0;
    
    gpio_value = gpiod_get_value(test_gpio);
    len = sprintf(value_str, "%d\n", gpio_value);
    
    if (copy_to_user(buffer, value_str, len))
        return -EFAULT;
    
    *pos += len;
    return len;
}

static const struct proc_ops gpio_proc_ops = {
    .proc_read = gpio_proc_read,
    .proc_write = gpio_proc_write,
};

static int gpio_test_probe(struct platform_device *pdev)
{
    int ret;
    
    pr_info("GPIO Test Driver: Probing device\n");
    
    // Get GPIO descriptor from device tree
    test_gpio = devm_gpiod_get(&pdev->dev, "test", GPIOD_OUT_LOW);
    if (IS_ERR(test_gpio)) {
        pr_err("Failed to get GPIO descriptor: %ld\n", PTR_ERR(test_gpio));
        return PTR_ERR(test_gpio);
    }
    
    // Set initial direction and value
    ret = gpiod_direction_output(test_gpio, 0);
    if (ret) {
        pr_err("Failed to set GPIO direction: %d\n", ret);
        return ret;
    }
    
    // Create proc entry for easy testing
    proc_entry = proc_create(PROC_ENTRY_NAME, 0666, NULL, &gpio_proc_ops);
    if (!proc_entry) {
        pr_err("Failed to create proc entry\n");
        return -ENOMEM;
    }
    
    pr_info("GPIO Test Driver: Successfully probed, GPIO ready\n");
    return 0;
}

static int gpio_test_remove(struct platform_device *pdev)
{
    pr_info("GPIO Test Driver: Removing device\n");
    
    if (proc_entry) {
        proc_remove(proc_entry);
    }
    
    // GPIO is automatically released due to devm_gpiod_get
    
    return 0;
}

static const struct of_device_id gpio_test_of_match[] = {
    { .compatible = "custom,gpio-test" },
    { }
};
MODULE_DEVICE_TABLE(of, gpio_test_of_match);

static struct platform_driver gpio_test_driver = {
    .probe = gpio_test_probe,
    .remove = gpio_test_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = gpio_test_of_match,
    },
};

static int __init gpio_test_init(void)
{
    pr_info("GPIO Test Driver: Initializing\n");
    return platform_driver_register(&gpio_test_driver);
}

static void __exit gpio_test_exit(void)
{
    pr_info("GPIO Test Driver: Exiting\n");
    platform_driver_unregister(&gpio_test_driver);
}

module_init(gpio_test_init);
module_exit(gpio_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Batman");
MODULE_DESCRIPTION("GPIO Test Driver for RPi 4");
MODULE_VERSION("1.0");