#include<linux/module.h>
#include<linux/init.h>
#include<linux/gpio.h>
#include<linux/device.h>
#include<linux/cdev.h>
#include<linux/fs.h>


#define DEVICE_NAME "test_gpio"
#define CLASS_NAME "gpio_test"
#define GPIO_PIN    17

static int major_number;
static struct class* gpio_class = NULL;
static struct device* gpio_device = NULL;
static bool gpio_state = false;

//function prototypes
static int device_open(struct inode*, struct file*);
static int device_release(struct inode*, struct file*);
static ssize_t device_read(struct file*, char*, size_t, loff_t*);
static ssize_t device_write(struct file*, const char*, size_t, loff_t*);

static struct  file_operations fops = {
    .open = device_open,
    .read = device_read,
    .write = device_write,
    .release = device_release,
};

static int __init gpio_driver_init(void){
    printk(KERN_INFO, "GPIO Driver: initializing\n");
    
    //Register major number 
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if( major_number < 0){
        printk(KERN_ALERT, "GPIO Driver: could not get a major number \n");
        return major_number;
    }

    printk(KERN_INFO, "GPIO Driver: got major number %d\n", major_number);

    //first we create the class needed to create a device to finally be able to acces to them trhough /dev/
    gpio_device = device_create(gpio_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(gpio_device)) {
        class_destroy(gpio_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ALERT "GPIO Driver: Failed to create device\n");
        return PTR_ERR(gpio_device);
    }
    printk(KERN_INFO "GPIO Driver: Device created successfully\n");

    // Request GPIO pin to the kernel
    if (gpio_request(GPIO_PIN, "test_gpio_pin")) {
        printk(KERN_ALERT "GPIO Driver: Cannot request GPIO %d\n", GPIO_PIN);
        device_destroy(gpio_class, MKDEV(major_number, 0));
        class_destroy(gpio_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return -EBUSY;
    }

    // Set GPIO direction to output
    gpio_direction_output(GPIO_PIN, 0);
    gpio_set_value(GPIO_PIN, 0);
    
    printk(KERN_INFO "GPIO Driver: GPIO %d configured as output\n", GPIO_PIN);
    printk(KERN_INFO "GPIO Driver: Module loaded successfully\n");
    return 0;

}

static void __exit gpio_driver_exit(void){
    gpio_set_value(GPIO_PIN, 0);  // Turn off GPIO before exit
    gpio_free(GPIO_PIN);
    device_destroy(gpio_class, MKDEV(major_number, 0));
    class_destroy(gpio_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "GPIO Driver: Module unloaded successfully\n");
}

// Device file operations
static int device_open(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "GPIO Driver: Device opened\n");
    return 0;
}

static ssize_t device_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    char state_str[10];
    int bytes_read = 0;
    
    // Read current GPIO state
    gpio_state = gpio_get_value(GPIO_PIN);
    snprintf(state_str, sizeof(state_str), "%d\n", gpio_state ? 1 : 0);
    bytes_read = strlen(state_str);
    
    if (*offset >= bytes_read) {
        return 0;  // EOF
    }
    
    if (copy_to_user(buffer, state_str, bytes_read)) {
        return -EFAULT;
    }
    
    *offset += bytes_read;
    printk(KERN_INFO "GPIO Driver: Read GPIO state: %d\n", gpio_state ? 1 : 0);
    return bytes_read;
}

static ssize_t device_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    char *data;
    int value;
    
    data = kmalloc(len + 1, GFP_KERNEL);
    if (!data) {
        return -ENOMEM;
    }
    
    if (copy_from_user(data, buffer, len)) {
        kfree(data);
        return -EFAULT;
    }
    
    data[len] = '\0';
    
    // Parse the input (expecting "0" or "1")
    if (kstrtoint(data, 10, &value) == 0) {
        gpio_state = (value != 0);
        gpio_set_value(GPIO_PIN, gpio_state);
        printk(KERN_INFO "GPIO Driver: Set GPIO %d to %d\n", GPIO_PIN, gpio_state ? 1 : 0);
    } else {
        printk(KERN_WARNING "GPIO Driver: Invalid input, expected 0 or 1\n");
        kfree(data);
        return -EINVAL;
    }
    
    kfree(data);
    return len;
}

static int device_release(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "GPIO Driver: Device closed\n");
    return 0;
}

module_init(gpio_driver_init);
module_exit(gpio_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Superman");
MODULE_DESCRIPTION("Test GPIO Driver for Raspberry Pi 4");
MODULE_VERSION("1.0");