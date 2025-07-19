#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/cdev.h>

/* Driver name and description */
#define DRIVER_NAME     "custom-gpio-driver"
#define DEVICE_NAME     "simplegpio"
#define CLASS_NAME      "gpio_class"

/* Private data structure */
struct custom_gpio_data {
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;
    struct gpio_desc *gpio;
};

// File operations prototypes
static int gpio_open(struct inode *inode, struct file *file);
static int gpio_release(struct inode *inode, struct file *file);
static ssize_t gpio_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static ssize_t gpio_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset);

// File operations structure
static struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .open = gpio_open,
    .release = gpio_release,
    .read = gpio_read,
    .write = gpio_write,
};

static int gpio_open(struct inode *inode, struct file *file){
    struct custom_gpio_data* data = container_of(inode->i_cdev, struct custom_gpio_data, cdev);
    file->private_data = data;
    pr_info("Custom GPIO device opened\n");
    return 0;
}

static int gpio_release(struct inode *inode, struct file *file)
{
    pr_info("Custom GPIO device closed\n");
    return 0;
}

static ssize_t gpio_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset){
    struct custom_gpio_data* data = file->private_data;

    char kbuf[16];
    int value;

    if (len > sizeof(kbuf) - 1)
        return -EINVAL;

    if (copy_from_user(kbuf, buffer, len))
        return -EFAULT;

    kbuf[len] = '\0';
    sscanf(kbuf, "%d", &value);
    pr_info("You write %d to GPIO LED\n", value);

    /* Validate input: expect "0" or "1" */
    if ( !(value == 0 || value == 1) )
        return -EINVAL;
    
    /* Set GPIO value */
    gpiod_set_value(data->gpio, value);
    
    dev_info(data->dev, "GPIO set to %c\n", value);

    return len;
}

static ssize_t gpio_read(struct file *file, char __user *buffer, size_t len, loff_t *offset){
    struct custom_gpio_data* data = file->private_data;
    char kbuf[16];
    int value, read_len;

    if(*offset > 0)
        return 0;
    
    value = gpiod_get_value(data->gpio);
    read_len = snprintf(kbuf, sizeof(kbuf), "Val=%d\n", value);

    if(copy_to_user(buffer, kbuf, read_len))
        return -EFAULT;

    *offset+=read_len;
    return read_len;

}

/* Probe function - called when device is matched */
static int custom_gpio_probe(struct platform_device *pdev)
{
    struct custom_gpio_data *data;
    int ret;

    dev_info(&pdev->dev, "Custom GPIO driver probe started\n");

    /* Allocate memory for private data */
    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->dev = &pdev->dev;

    /* Get GPIO from device tree */
    data->gpio = devm_gpiod_get(&pdev->dev, NULL, GPIOD_OUT_LOW);
    if (IS_ERR(data->gpio)) {
        ret = PTR_ERR(data->gpio);
        dev_err(&pdev->dev, "Failed to get GPIO: %d\n", ret);
        return ret;
    }

    /* Set GPIO direction (should already be output from DT, but explicit is better) */
    ret = gpiod_direction_output(data->gpio, 0);
    if (ret) {
        dev_err(&pdev->dev, "Failed to set GPIO direction: %d\n", ret);
        return ret;
    }

    // Allocate device number (device, minor_number, number of devoces, name)
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate device number: %d\n", ret);
        return ret;
    }

    pr_info("Device number allocated: Major=%d, Minor=%d\n", MAJOR(data->dev_num), MINOR(data->dev_num));

    // Initialize character device
    cdev_init(&data->cdev, &gpio_fops);
    data->cdev.owner = THIS_MODULE;

    // Add character device to system
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to add character device: %d\n", ret);
        goto cleanup_chrdev;
    }

    // Create device class
    data->class = class_create(CLASS_NAME);
    if (IS_ERR(data->class)) {
        ret = PTR_ERR(data->class);
        pr_err("Failed to create device class: %d\n", ret);
        goto cleanup_cdev;
    }

    // Create device file
    data->dev = device_create(data->class, NULL, data->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(data->dev)) {
        ret = PTR_ERR(data->dev);
        pr_err("Failed to create device: %d\n", ret);
        goto cleanup_class;
    }

    pr_info("GPIO driver initialized successfully. Device: /dev/%s\n", DEVICE_NAME);


    /* Store private data in platform device */
    platform_set_drvdata(pdev, data);

    dev_info(&pdev->dev, "Custom GPIO driver probe completed successfully\n");
    dev_info(&pdev->dev, "GPIO pin %d initialized as output\n", desc_to_gpio(data->gpio));
    dev_info(&pdev->dev, "Device created: /dev/%s \n", DEVICE_NAME);

    return 0;

cleanup_class:
    class_destroy(data->class);
cleanup_cdev:
    cdev_del(&data->cdev);
cleanup_chrdev:
    unregister_chrdev_region(data->dev_num, 1);
    return ret;
}

/* Remove function - called when device is removed */
static void custom_gpio_remove(struct platform_device *pdev)
{
    struct custom_gpio_data *data = platform_get_drvdata(pdev);
    dev_info(&pdev->dev, "Custom GPIO driver remove started\n");

    /* Set GPIO to safe state before removing */
    if (data->gpio) {
        gpiod_set_value(data->gpio, 0);
        dev_info(&pdev->dev, "GPIO set to low before removal\n");
    }

    /* GPIO will be automatically freed by devm_gpiod_get */

    // Clean up device
    device_destroy(data->class, data->dev_num);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_num, 1);

    dev_info(&pdev->dev, "Custom GPIO driver removed successfully\n");

}

/* Device tree matching table */
static const struct of_device_id custom_gpio_of_match[] = {
    { .compatible = "custom,gpio-driver", },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, custom_gpio_of_match);

/* Platform driver structure */
static struct platform_driver custom_gpio_driver = {
    .probe = custom_gpio_probe,
    .remove = custom_gpio_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = custom_gpio_of_match,
    },
};

/* Module initialization */
static int __init custom_gpio_init(void)
{
    pr_info("Custom GPIO driver loading...\n");
    return platform_driver_register(&custom_gpio_driver);
}

/* Module cleanup */
static void __exit custom_gpio_exit(void)
{
    pr_info("Custom GPIO driver unloading...\n");
    platform_driver_unregister(&custom_gpio_driver);
}

/* Module macros */
module_init(custom_gpio_init);
module_exit(custom_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lalo");
MODULE_DESCRIPTION("GPIO device driver using DT Overlay");
MODULE_VERSION("1.0");
