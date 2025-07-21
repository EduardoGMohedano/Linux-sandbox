#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#define DRIVER_NAME     "custom-tft-screen"
#define DEVICE_NAME     "simple-tft"
#define CLASS_NAME      "tft_class"

struct tft_data{
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;
    struct spi_device* spi;
    struct gpio_desc *rst_pin; //Real struct to manage GPIO
};

// File operations prototypes
static int tft_open(struct inode *inode, struct file *file);
static int tft_release(struct inode *inode, struct file *file);
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
    struct tft_data* data = container_of(inode->i_cdev, struct tft_data, cdev);  
    file->private_data = data;
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
    int count = 0;

    if (*offset > 0) {
        return 0; // EOF
    }

    // Show status of all requested GPIOs
    msg_len = snprintf(msg, sizeof(msg), "GPIO Status:\n");

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
    return len;
}

static void tft_hardware_reset(struct tft_data* tft){
    if(tft->rst_pin){
        gpiod_set_value(tft->rst_pin, 0);
        msleep(100);
        gpiod_set_value(tft->rst_pin, 1);
        msleep(100);
    }
}

// In probe function
static int tft_probe(struct spi_device *spi){
    struct tft_data* data;
    int ret;

    dev_info(&spi->dev, "Probe driver has started\n");
    /*Allocate memory for dev structure*/
    data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
    if(!data)
        return -ENOMEM;

    //Save SPI device reference
    data->spi = spi;

    //Set driver data
    spi_set_drvdata(spi, data);

    //Set SPI transaction settings
    spi->mode = SPI_MODE_3;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);

    if(ret < 0){
        dev_err(&spi->dev, "SPI setup failed: %d\n", ret);
        return ret;
    }

    data->rst_pin = devm_gpiod_get(&spi->dev, "rst", GPIOD_OUT_HIGH);
    if (IS_ERR(data->rst_pin)) {
        ret = PTR_ERR(data->rst_pin);
        dev_err(&spi->dev, "Failed to get RST pin: %d\n", ret);
        return ret;
    }

    /* Set RST GPIO direction*/
    ret = gpiod_direction_output(data->rst_pin, 0);
    if (ret) {
        dev_err(&spi->dev, "Failed to set GPIO direction: %d\n", ret);
        return ret;
    }

    //Perform device reset
    tft_hardware_reset(data);

    //Command sequence to init TFT



    // Allocate device number (device, minor_number, number of devoces, name)
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&spi->dev,"Failed to allocate device number: %d\n", ret);
        return ret;
    }

    pr_info("Device number allocated: Major=%d, Minor=%d\n", MAJOR(data->dev_num), MINOR(data->dev_num));

    // Initialize character device
    cdev_init(&data->cdev, &tft_fops);
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

    dev_info(&spi->dev, "Custom TFT driver probe completed successfully\n");
    dev_info(&spi->dev, "SPI max frequency: %dHz\n", spi->max_speed_hz);
    dev_info(&spi->dev, "Device created: /dev/%s \n", DEVICE_NAME);
    return 0;

cleanup_class:
    class_destroy(data->class);
cleanup_cdev:
    cdev_del(&data->cdev);
cleanup_chrdev:
    unregister_chrdev_region(data->dev_num, 1);
    return ret;

}

static void tft_remove(struct spi_device *spi){
    struct tft_data* data = spi_get_drvdata(spi);
    dev_info(&spi->dev, "Custom TFT driver remove started\n");

    /* Set GPIO to safe state before removing */
    if (data->rst_pin) {
        gpiod_set_value(data->rst_pin, 1);
        dev_info(&spi->dev, "GPIO set to high before removal\n");
    }

    /* GPIO needs to be freed manually since we got them by using indexes */
    gpiod_put(data->rst_pin);

    // Clean up device
    device_destroy(data->class, data->dev_num);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_num, 1);

    dev_info(&spi->dev, "Custom GPIO driver removed successfully\n");
}

/* SPI Device ID table - this is what you asked about */
static const struct spi_device_id tft_spi_ids[] = {
    { "raio8875", 0 },
    { "simple-tft", 0 },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, tft_spi_ids);

static const struct of_device_id tft_of_match[] = {
    { .compatible = "raio,raio8875", },
    { }
};

MODULE_DEVICE_TABLE(of, tft_of_match);

static struct spi_driver tft_driver = {
    .probe = tft_probe,
    .remove = tft_remove,
    .id_table = tft_spi_ids,
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = tft_of_match,
    },
};

/* Module initialization */
static int __init tft_screen_init(void)
{
    pr_info("Custom TFT driver loading...\n");
    return spi_register_driver(&tft_driver);
}

/* Module cleanup */
static void __exit tft_screen_exit(void)
{
    pr_info("Custom TFT driver unloading...\n");
    spi_unregister_driver(&tft_driver);
}

/* Module macros */
module_init(tft_screen_init);
module_exit(tft_screen_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LALO");
MODULE_DESCRIPTION("TFT screen driver using /dev/ interface for RA8875");
MODULE_VERSION("1.0");
