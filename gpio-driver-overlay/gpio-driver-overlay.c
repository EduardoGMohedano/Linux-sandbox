#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>

/* Driver name and description */
#define DRIVER_NAME "custom-gpio-driver"

/* Private data structure */
struct custom_gpio_data {
    struct gpio_desc *gpio;
    struct device *dev;
};

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

    /* Store private data in platform device */
    platform_set_drvdata(pdev, data);

    /* Initialize your driver here */
    /* Example: Create sysfs files, setup timers, etc. */

    dev_info(&pdev->dev, "Custom GPIO driver probe completed successfully\n");
    dev_info(&pdev->dev, "GPIO pin %d initialized as output\n", desc_to_gpio(data->gpio));

    return 0;
}

/* Remove function - called when device is removed */
static void custom_gpio_remove(struct platform_device *pdev)
{
    struct custom_gpio_data *data = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "Custom GPIO driver remove started\n");

    /* Cleanup your driver here */
    /* Example: Remove sysfs files, stop timers, etc. */

    /* Set GPIO to safe state before removing */
    if (data->gpio) {
        gpiod_set_value(data->gpio, 0);
        dev_info(&pdev->dev, "GPIO set to low before removal\n");
    }

    /* GPIO will be automatically freed by devm_gpiod_get */

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