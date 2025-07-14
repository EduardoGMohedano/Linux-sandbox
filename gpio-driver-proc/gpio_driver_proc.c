#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/gpio/consumer.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>


#define LED_GPIO_PIN 	17  // GPIO pin for LED (change as needed)
#define LED_GPIO_OFFSET 512  // GPIO pin for LED (change as needed)
#define BLINK_INTERVAL 	1000  // Blink interval in milliseconds
#define MAX_SIZE_BUFF   1024

// static struct task_struct *blink_thread;
static int led_state = 0;
static int stop_thread = 0;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LALO");
MODULE_DESCRIPTION("Simple LED Blink Module for Raspberry Pi");
MODULE_VERSION("1.0");

static struct gpio_desc *led_gpio = NULL;
static struct proc_dir_entry* lll_proc = NULL;
static char data_buffer[MAX_SIZE_BUFF];

// Thread function that blinks the LED
// static int led_blink_func(void *data)
// {
//     printk(KERN_INFO "LED blink thread started\n");
    
//     while (!kthread_should_stop() && !stop_thread) {
//         // Toggle LED state
//         led_state = !led_state;
//         gpiod_set_value(led_gpio, led_state);
        
//         printk(KERN_INFO "LED state: %s\n", led_state ? "ON" : "OFF");
        
//         // Sleep for the specified interval
//         msleep(BLINK_INTERVAL);
//     }
    
//     // Turn off LED when thread exits
//     gpiod_set_value(led_gpio, 0);
//     printk(KERN_INFO "LED blink thread stopped\n");
    
//     return 0;
// }

ssize_t lll_read(struct file* file, char __user* user, size_t size, loff_t * off){
    static char msg[] = "Hello friend!\n";
    int len = strlen(msg);

    if(*off >= len)
	return 0;

    copy_to_user(user,msg,len);
    *off = len;

    return len ;
}

long int lll_write(struct file* file, const char __user* user, size_t size, loff_t * off){
    memset(data_buffer, 0x0, sizeof(data_buffer));
    
    if(size >= MAX_SIZE_BUFF)
        size = MAX_SIZE_BUFF;
    
    copy_from_user(data_buffer, user, size);

    printk("You said: %s\n", data_buffer);
    printk("You said at location 0th: %c\n", data_buffer[0]);

    if( data_buffer[0] == '0' )
        gpiod_set_value(led_gpio, 0);
    else if( data_buffer[0] == '1' )
        gpiod_set_value(led_gpio, 1);

    return size;
}

static const struct proc_ops lll_proc_fops ={
    .proc_read = lll_read,
    .proc_write = lll_write,
};

// Module initialization function
static int __init led_blink_init(void){
    int ret;
    
    printk(KERN_INFO "LED Blink: Initializing module\n");
    
    // First try the modern descriptor-based API
    led_gpio = gpio_to_desc( LED_GPIO_PIN + LED_GPIO_OFFSET);
    if (!led_gpio) {
           printk(KERN_INFO "LED Blink: Error getting pin\n");
	   return -ENODEV;
    }
    
    // Set GPIO direction to output
    ret = gpiod_direction_output(led_gpio, 0);
    if (ret) {
        printk(KERN_ERR "LED Blink: Failed to set GPIO direction\n");
        return ret;
    }
    
    gpiod_set_value(led_gpio, 0);
    
    lll_proc = proc_create("lll-gpio", 0666, NULL, &lll_proc_fops);
    if(!lll_proc){
        gpiod_put(led_gpio);
        printk(KERN_ALERT "lll_proc could not be created\n");
        return -1;
    }

    // Create and start the kernel thread
    // blink_thread = kthread_run(led_blink_func, NULL, "led_blink_thread");
    // if (IS_ERR(blink_thread)) {
    //     printk(KERN_ERR "Failed to create kernel thread\n");
    //     gpiod_put(led_gpio);
    //     return PTR_ERR(blink_thread);
    // }
    
    return 0;
}

// Module cleanup function
static void __exit led_blink_exit(void){
    // Turn off LED and free GPIO
    
    // Stop the kernel thread
    // stop_thread = 1;
    // if (blink_thread) {
    //     kthread_stop(blink_thread);
    //     blink_thread = NULL;
    // }

    gpiod_set_value(led_gpio, 0);
    gpiod_put(led_gpio);
    proc_remove(lll_proc);
    printk(KERN_INFO "LED Blink: Module unloaded successfully\n");
}

// Register module entry and exit points
module_init(led_blink_init);
module_exit(led_blink_exit);
