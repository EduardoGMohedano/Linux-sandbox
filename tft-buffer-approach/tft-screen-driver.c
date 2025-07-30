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
#include "ra8875.h"

#define DRIVER_NAME         "custom-tft-screen"
#define DEVICE_NAME         "simple-tft"
#define CLASS_NAME          "tft_class"
#define BOUNCE_INTERVAL     350
#define TEXT_STEP_PIXELS    20

/* Color structure for RGB values */
struct tft_color {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
};

struct Point{
    int x;
    int y;
};

#define TFT_STRUCT_TO_U16(color)    ( (((u16)color.red&31) << 11) | (((u16)color.green&63) << 5) | (((u16)color.blue&31) << 0) )

//ioctl commands definition
#define TFT_IOC_MAGIC         'T'
#define TFT_SET_TEXT_COLOR    _IOW(TFT_IOC_MAGIC, 1, struct tft_color)
#define TFT_GET_TEXT_COLOR    _IOR(TFT_IOC_MAGIC, 2, struct tft_color)
#define TFT_SET_BG_COLOR      _IOW(TFT_IOC_MAGIC, 3, struct tft_color)
#define TFT_RESET_COLORS      _IO(TFT_IOC_MAGIC, 4)
#define TFT_SET_TIME_INTER    _IOW(TFT_IOC_MAGIC, 5, int)

struct tft_data{
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;
    struct spi_device* spi;
    struct gpio_desc *rst_pin; //Real struct to manage GPIO
};

static struct Point draw_area[4] = {0};
u16 draw_buffer[25];
u8 draw_buffer8[50];

/* Driver state */
static struct tft_color current_text_color = {31, 0, 31}; /* Default red */
static struct tft_color current_bg_color = {0x00, 0x00, 0x00};   /* Default black */

static struct task_struct *bounce_thread;
static int stop_thread = 0;
char text_buff[64] = "Hello world!";
int text_len = 13;
int time_interval = BOUNCE_INTERVAL;

// File operations prototypes
static int tft_open(struct inode *inode, struct file *file);
static int tft_release(struct inode *inode, struct file *file);
static ssize_t tft_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static ssize_t tft_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset);
static long tft_ioctl(struct file*, unsigned int, unsigned long);

/*TFT related functions*/
uint8_t ra8875_init(struct tft_data* data);
void ra8875_enable_display(struct spi_device* spi, bool enable);
void ra8875_configure_clocks( struct spi_device* spi, bool high_speed);
void ra8875_set_window(struct spi_device* spi, uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye);
void ra8875_set_memory_write_cursor(struct spi_device* spi, uint16_t x, uint16_t y);
void configurePWM(struct spi_device* spi, uint8_t pwm_pin, bool enable, uint8_t pwm_clock);
void PWMout(struct spi_device* spi, uint8_t pwm_pin, uint8_t duty_cycle);

uint8_t ra8875_read_register(struct spi_device* spi, uint8_t reg);
void ra8875_write_register(struct spi_device* spi, uint8_t reg, uint8_t value);
void writeCommand(struct spi_device* spi ,uint8_t d);
uint8_t readData(struct spi_device* spi);
void writeData(struct spi_device* spi, uint8_t d);
uint8_t spi_send_t(struct spi_device* spi, uint8_t data, uint8_t data2);
int spi_send_buff(struct spi_device *spi, u16 *data, int count);
int send_bulk_spi(struct spi_device *spi, u16 *data, int count);
uint8_t spi_send_read_t(struct spi_device* spi, u8 data, u8 data2);
/*TFT related functions*/

/**********************
*** TFT NORMAL PROTOTYPES
**********************/
void fillScreen(struct spi_device* spi, uint16_t color);
void setCursor(struct spi_device* spi,uint16_t x, uint16_t y);
void textTransparent(struct spi_device* spi,uint16_t foreColor);
void textWrite(struct spi_device* spi,const char *buffer, uint16_t len);
void textMode(struct spi_device* spi);
void textEnlarge(struct spi_device* spi,uint8_t scale);
void graphicsMode(struct spi_device* spi);


void fillScreen(struct spi_device* spi,uint16_t color) {
    // rectHelper(0, 0, _width - 1, _height - 1, color, true);

    /* Set X */
    int x = 0;
    ra8875_write_register(spi, 0x91, x);
    ra8875_write_register(spi, 0x92, x>>8);

    /* Set Y */
    int y = 0;
    ra8875_write_register(spi, 0x93, y);
    ra8875_write_register(spi, 0x94, y>>8);

    /* Set X1 */
    int w = 799;
    ra8875_write_register(spi, 0x95, w);
    ra8875_write_register(spi, 0x96, w>>8);

    /* Set Y1 */
    int h = 479;
    ra8875_write_register(spi, 0x97, h);
    ra8875_write_register(spi, 0x98, h>>8);

    /* Set Color */
    ra8875_write_register(spi, 0x63, (color & 0xf800) >> 11);
    ra8875_write_register(spi, 0x64, (color & 0x07e0) >> 5);
    ra8875_write_register(spi, 0x65, (color & 0x001f));
    
    /* Draw! */
    ra8875_write_register(spi, 0x90, 0xB0);    
    msleep(150);
}

void setCursor(struct spi_device*spi, uint16_t x, uint16_t y){
    /* Set cursor location */
    writeCommand(spi, 0x2A);
    writeData(spi, x & 0xFF);
    writeCommand(spi, 0x2B);
    writeData(spi, x >> 8);
    writeCommand(spi, 0x2C);
    writeData(spi, y & 0xFF);
    writeCommand(spi, 0x2D);
    writeData(spi, y >> 8);
}
  
void textTransparent(struct spi_device* spi, uint16_t foreColor) {
    /* Set Fore Color */
    writeCommand(spi, 0x63);
    writeData(spi, (foreColor & 0xf800) >> 11);
    writeCommand(spi, 0x64);
    writeData(spi, (foreColor & 0x07e0) >> 5);
    writeCommand(spi, 0x65);
    writeData(spi, (foreColor & 0x001f));

    /* Set transparency flag */
    writeCommand(spi, 0x22);
    uint8_t temp = readData(spi);
    temp |= (1 << 6); // Set bit 6
    writeData(spi, temp);
}
  
void textWrite(struct spi_device* spi, const char *buffer, uint16_t len) {
    if (len == 0)
        len = sizeof(buffer);
    writeCommand(spi, 0x02);
    for (uint16_t i = 0; i < len; i++)
        writeData(spi, buffer[i]);
  
    msleep(10);
}
  
void textMode(struct spi_device* spi) {
    /* Set text mode */
    writeCommand(spi, 0x40);
    uint8_t temp = readData(spi);
    temp |= 0x80; // Set bit 7
    writeData(spi, temp);
  
    /* Select the internal (ROM) font*/
    writeCommand(spi, 0x21);
    temp = readData(spi);
    temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
    writeData(spi, temp);
}

void textEnlarge(struct spi_device* spi, uint8_t scale) {
    if (scale > 3)
        scale = 3; // highest setting is 3
  
    /* Set font size flags */
    writeCommand(spi, 0x22);
    uint8_t temp = readData(spi);
    temp &= ~(0xF); // Clears bits 0..3
    temp |= scale << 2;
    temp |= scale;
  
    writeData(spi, temp);
    // _textScale = scale;
}

void graphicsMode(struct spi_device* spi) {
    /* Set text mode */
    writeCommand(spi, 0x40);
    uint8_t temp = readData(spi);
    temp &= 0x7F; // clear bit 7
    writeData(spi, temp);
}

/*Native SPI functions*/

int send_bulk_spi(struct spi_device *spi, u16 *data, int count){
    struct spi_message msg;
    struct spi_transfer *transfers;
    int i, ret;
    
    transfers = kcalloc(count, sizeof(struct spi_transfer), GFP_KERNEL);
    if (!transfers)
        return -ENOMEM;
    
    spi_message_init(&msg);
    
    for (i = 0; i < count; i++) {
        transfers[i].tx_buf = &data[i];
        transfers[i].len = 2;  // 16 bits = 2 bytes
        transfers[i].bits_per_word = 16;
        transfers[i].speed_hz = spi->max_speed_hz;
        
        spi_message_add_tail(&transfers[i], &msg);
    }
    
    ret = spi_sync(spi, &msg);
    
    kfree(transfers);
    return ret;
}

int spi_send_buff(struct spi_device *spi, u16 *data, int count){
    struct spi_message msg;
    struct spi_transfer transfer = {0};
    int ret;
    
    draw_buffer8[0] = 0; //maybe this zero needs to be send separately and assert CS pin first
    for(int i = 1; i <= count; i++){
        draw_buffer8[(2*i)-1] = (u8)(draw_buffer[i] >> 8);
        draw_buffer8[(2*i)] = (u8)(draw_buffer[i]);
    }
    
    transfer.tx_buf = draw_buffer8;
    transfer.len = (count * 2) + 1;  // count * 2 bytes per 16-bit word
    transfer.bits_per_word = 8;
    transfer.speed_hz = spi->max_speed_hz;
    transfer.cs_change = 0;
    
    spi_message_init(&msg);
    spi_message_add_tail(&transfer, &msg);
    
    ret = spi_sync(spi, &msg);
    if( ret < 0 ){
        pr_err("SPI send buff transaction failed\n");
        return ret;
    }
    return ret;
}

uint8_t spi_send_t(struct spi_device* spi, uint8_t data, uint8_t data2){
    u8 data_buff[2] = {data, data2};
    int ret;

    ret = spi_write(spi, data_buff, 2);
    if( ret < 0 ){
        pr_err("SPI transaction failed\n");
        return ret;
    }
    return 0;
}

uint8_t spi_send_read_t(struct spi_device* spi, u8 data, u8 data2){
    struct spi_transfer xfer = {};
    struct spi_message msg;
    u8 data_buff[2] = {data, data2};
    u8 data_rx[2] = {0,0};
    int ret;

    xfer.tx_buf = data_buff;
    xfer.rx_buf = data_rx;
    xfer.len = 2;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(spi, &msg);

    // ret = spi_write_then_read(spi, data_buff, 0, data_rx, 2);
    if( ret != 0 ){
        pr_err("SPI transaction failed\n");
        return ret;
    }

    // pr_info("Received rx data: %02x %02x", data_rx[0], data_rx[1]);
    return data_rx[1];
}

void writeCommand(struct spi_device* spi, uint8_t d){
    spi_send_t(spi, (uint8_t)RA8875_MODE_CMD_WRITE, d);
}

uint8_t readData(struct spi_device* spi){
    uint8_t val = 0;
    val = spi_send_read_t(spi, (u8)RA8875_MODE_DATA_READ, 0);
    return val;
}

void writeData(struct spi_device* spi, uint8_t d){
    spi_send_t(spi, (uint8_t)RA8875_MODE_DATA_WRITE, d);
}

uint8_t ra8875_read_register(struct spi_device* spi, uint8_t reg){
    writeCommand(spi, reg);
    uint8_t rcv_buf = readData(spi);
    return rcv_buf;
}

void ra8875_write_register(struct spi_device* spi, uint8_t reg, uint8_t value){
    writeCommand(spi, reg);
    writeData(spi, value);
}

void configurePWM(struct spi_device* spi, uint8_t pwm_pin, bool enable, uint8_t pwm_clock){
    uint8_t register_pin = (pwm_pin == PWM_PIN_1) ? RA8875_REG_PC1R : RA8875_REG_PC2R;
    if( enable ){
        ra8875_write_register(spi, register_pin, 0x80 | (pwm_clock & 0xF));
    }
    else{
        ra8875_write_register(spi, register_pin, 0x00 | (pwm_clock & 0xF));
    }
}

void PWMout(struct spi_device* spi, uint8_t pwm_pin, uint8_t duty_cycle){
    uint8_t register_pin = (pwm_pin == PWM_PIN_1) ? RA8875_REG_P1DCR : RA8875_REG_P2DCR;
    
    ra8875_write_register(spi, register_pin, duty_cycle);
}

uint8_t ra8875_init(struct tft_data* data){
    struct {
        uint8_t cmd;                                   // Register address of command
        uint8_t data;                                  // Value to write to register
    } init_cmds[] = {
        {RA8875_REG_SYSR,   SYSR_VAL},                 // System Configuration Register (SYSR)
        {RA8875_REG_PCSR,   PCSR_VAL},
        {RA8875_REG_HDWR,   HDWR_VAL},                 // LCD Horizontal Display Width Register (HDWR)
        {RA8875_REG_HNDFTR, HNDFTR_VAL},               // Horizontal Non-Display Period Fine Tuning Option Register (HNDFTR)
        {RA8875_REG_HNDR,   HNDR_VAL},                 // Horizontal Non-Display Period Register (HNDR) TODO CORRECT FORMULA COULD BE 3 BY DEFA
        {RA8875_REG_HSTR,   HSTR_VAL},                 // HSYNC Start Position Register (HSTR)
        {RA8875_REG_HPWR,   HPWR_VAL},                 // HSYNC Pulse Width Register (HPWR)
        {RA8875_REG_VDHR0,  VDHR_VAL & 0x0FF},         // LCD Vertical Display Height Register (VDHR0)
        {RA8875_REG_VDHR1,  VDHR_VAL >> 8},            // LCD Vertical Display Height Register0 (VDHR1)
        {RA8875_REG_VNDR0,  VNDR_VAL & 0x0FF},         // LCD Vertical Non-Display Period Register (VNDR0)
        {RA8875_REG_VNDR1,  VNDR_VAL >> 8},            // LCD Vertical Non-Display Period Register (VNDR1)
        {RA8875_REG_VSTR0,  VSTR_VAL & 0x0FF},         // VSYNC Start Position Register (VSTR0)
        {RA8875_REG_VSTR1,  VSTR_VAL >> 8},            // VSYNC Start Position Register (VSTR1)
        {RA8875_REG_VPWR,   VPWR_VAL}                 // VSYNC Pulse Width Register (VPWR)
    };
    
    uint8_t init_cmd_size  = sizeof(init_cmds)/sizeof(init_cmds[0]);

    pr_info("Initializing RA8875...");
    
    // Reset the RA8875
    pr_info("Sending reset sequence on pin...");
    gpiod_set_value(data->rst_pin, 0);
    msleep(100);
    gpiod_set_value(data->rst_pin, 1);
    msleep(10);
    
    if ( ra8875_read_register(data->spi, 0x00) != 0x75 ){
        gpiod_put(data->rst_pin);
        pr_err("RA8875 screen was not found");
        return true;
    }

    pr_info("RA8875 screen found");
    
    //Initialize PLL clocks
    ra8875_configure_clocks(data->spi, true);

    // Send all the commands to init the display
    for (uint8_t i = 0; i < init_cmd_size; i++) {
        ra8875_write_register(data->spi, init_cmds[i].cmd, init_cmds[i].data);
    }

    //Set window area for the first time
    ra8875_set_window(data->spi, 0, LV_HOR_RES_MAX, 0, LV_VER_RES_MAX);

    // Perform a memory clear (wait maximum of 100 ticks) //could just be a delay 
    ra8875_write_register(data->spi, RA8875_REG_MCLR, 0x80);
    for(uint8_t i = 100; i != 0; i--) {
        if ((ra8875_read_register(data->spi, RA8875_REG_MCLR) & 0x80) == 0x00) {
            pr_info("Waiting for Memory clear to be finished...");
            break;
        }
        msleep(10);
    }
    
    msleep(250);

    // Enable the display
    ra8875_enable_display(data->spi, true);

#ifdef BACKLIGHT_INTERNAL
    ra8875_write_register(data->spi, RA8875_GPIOX, 1); //Enable pin attached to GPIOX as output to enable PWM
    configurePWM(data->spi, PWM_PIN_1, true, RA8875_PWM_CLK_DIV32);
    PWMout( data->spi, PWM_PIN_1, 255);
#endif

    return true;
}

void ra8875_enable_display(struct spi_device* spi, bool enable){
    pr_info("%s display \n", enable ? "Enabling" : "Disabling" );
    uint8_t val = enable ? 0x80 : 0x00;
    ra8875_write_register(spi, RA8875_REG_PWRR, val);            // Power and Display Control Register (PWRR)
}

void ra8875_configure_clocks(struct spi_device* spi, bool high_speed){
    uint8_t val;
    
    val = high_speed ? ((CONFIG_LV_DISP_RA8875_PLLDIVM << 7) | CONFIG_LV_DISP_RA8875_PLLDIVN) : 0x07;
    ra8875_write_register(spi, RA8875_REG_PLLC1, val);           // PLL Control Register 1 (PLLC1)
    msleep(5);
    
    val = high_speed ? CONFIG_LV_DISP_RA8875_PLLDIVK : 0x03;
    ra8875_write_register(spi, RA8875_REG_PLLC2, val);           // PLL Control Register 2 (PLLC2)
    msleep(5);
}

//Set a drawing window area
void ra8875_set_window(struct spi_device* spi, uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye){
    ra8875_write_register(spi, RA8875_REG_HSAW0, (uint8_t)(xs & 0x00FF)); // Horizontal Start Point 0 of Active Window (HSAW0)
    ra8875_write_register(spi, RA8875_REG_HSAW1, (uint8_t)(xs >> 8));    // Horizontal Start Point 1 of Active Window (HSAW1)
    ra8875_write_register(spi, RA8875_REG_VSAW0, (uint8_t)(ys & 0x00FF)); // Vertical Start Point 0 of Active Window (VSAW0)
    ra8875_write_register(spi, RA8875_REG_VSAW1, (uint8_t)(ys >> 8));    // Vertical Start Point 1 of Active Window (VSAW1)
    ra8875_write_register(spi, RA8875_REG_HEAW0, (uint8_t)(xe & 0x00FF)); // Horizontal End Point 0 of Active Window (HEAW0)
    ra8875_write_register(spi, RA8875_REG_HEAW1, (uint8_t)(xe >> 8));    // Horizontal End Point 1 of Active Window (HEAW1)
    ra8875_write_register(spi, RA8875_REG_VEAW0, (uint8_t)(ye & 0x00FF)); // Vertical End Point of Active Window 0 (VEAW0)
    ra8875_write_register(spi, RA8875_REG_VEAW1, (uint8_t)(ye >> 8));    // Vertical End Point of Active Window 1 (VEAW1)
}

// Used to set the cursor at certain position
void ra8875_set_memory_write_cursor(struct spi_device* spi, uint16_t x, uint16_t y){
    ra8875_write_register(spi, RA8875_REG_CURH0, (uint8_t)(x & 0x00FF));  // Memory Write Cursor Horizontal Position Register 0 (CURH0)
    ra8875_write_register(spi, RA8875_REG_CURH1, (uint8_t)(x >> 8));     // Memory Write Cursor Horizontal Position Register 1 (CURH1)
    ra8875_write_register(spi, RA8875_REG_CURV0, (uint8_t)(y & 0x00FF));  // Memory Write Cursor Vertical Position Register 0 (CURV0)
    ra8875_write_register(spi, RA8875_REG_CURV1, (uint8_t)(y >> 8));     // Memory Write Cursor Vertical Position Register 1 (CURV1)
}


/*Thread showing text on the screen*/
static int text_bounce_func(void *ptr_data){
// static int text_bounce_func(struct tft_data* data){
    printk(KERN_INFO "Rectangle bounce thread started\n");
    int position_x = 0;
    int position_y = 0;
    int direction_x = 1;
    int direction_y = 1;

    struct tft_data* data = (struct tft_data*)ptr_data;
    
    for(int i = 0; i < 25; i++)
        draw_buffer[i] = (u16)TFT_STRUCT_TO_U16(current_text_color);
        
    graphicsMode(data->spi);
    msleep(20); /* let this time pass */
    
    while (!kthread_should_stop() && !stop_thread) {
        //Make screen black and show text
        fillScreen(data->spi, 0x0000);
        // curr_color = TFT_STRUCT_TO_U16(current_text_color);
        printk(KERN_INFO "Rect positions Start:(%d,%d) End:(%d,%d)\n", position_x, position_y, position_x+4, position_y+4);
        ra8875_set_window(data->spi, position_x, position_x+4, position_y, position_y+4);        
        ra8875_set_memory_write_cursor(data->spi, position_x, position_y);

        writeCommand(data->spi, RA8875_REG_MRWC);

        spi_send_buff(data->spi, draw_buffer, 25);

        // Sleep for the specified interval
        msleep(time_interval);

        position_x+=direction_x * TEXT_STEP_PIXELS;
        position_y+=direction_y * TEXT_STEP_PIXELS;
        if( position_x + (17*text_len) >= LV_HOR_RES_MAX )
            direction_x = -1;
        if( position_x <= 0 )
            direction_x = 1;
        if( position_y + 30 >= LV_VER_RES_MAX )
            direction_y = -1;
        if( position_y <= 0 )
            direction_y = 1;
    }
    
    printk(KERN_INFO "Rectangle bounce thread finished\n");
    
    return 0;
}

// File operations structure
static struct file_operations tft_fops = {
    .owner = THIS_MODULE,
    .open = tft_open,
    .release = tft_release,
    .read = tft_read,
    .write = tft_write,
    .unlocked_ioctl = tft_ioctl,
};

static int tft_open(struct inode *inode, struct file *file){
    pr_info("TFT device opened\n");

    struct tft_data* data;
    data = container_of(inode->i_cdev, struct tft_data, cdev);  

    if(!data->spi){
        pr_err("SPI device is NULL\n");
        return -ENODEV;
    }
    
    file->private_data = data;
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
    struct tft_data* data = NULL;
    char cmd[128];
    char first_buff[32];
    int read_s;

    if (len >= sizeof(cmd)) {
        return -EINVAL;
    }
    
    data = (struct tft_data*)file->private_data;
    if(!data){
        pr_err("Driver structure is NULL\n");
        return -ENODEV;
    }
    
    if(!data->spi){
        pr_err("SPI device is NULL\n");
        return -ENODEV;
    }

    if( len > 0 ){
        if (copy_from_user(cmd, buffer, len)) {
            return -EFAULT;
        }
        cmd[len] = '\0';
        
        if( strncmp(cmd,"len", 3) == 0 ){
            //Receive buffer data
            // char *first_comma = strchr(cmd, ',');
            // if(!first_comma)
            //     return 0;
                
            // read_s = first_comma - cmd;  
            // first_buff[read_s+1] = '\0';
            // memcpy(first_buff, cmd, read_s);
            // int buffer_size = 0;
            // sscanf(first_buff, "len=%d", &buffer_size);

            // draw_buffer = (int*) kmalloc(1024, GFP_KERNEL);
            // if (!draw_buffer) {
            //     printk(KERN_ERR "Failed to allocate small buffer\n");
            //     return -ENOMEM;
            // }

            // //prepare a NEW DRAW BUFFER AND PARSE the new values

            // char* buffer_copy = strdup(cmd);  // Make a copy since strtok modifies the string
            // char* token;
            // char* saveptr;  // For thread-safe strtok_r
            // int count = 0;
            
            // if (buffer_copy == NULL) {
            //     return -1;  // Memory allocation failed
            // }
            
            // // Use strtok_r
            // token = strtok_r(buffer_copy, ",", &saveptr);
            
            // while (token != NULL && count < buffer_size) {
            //     // Convert string to integer
            //     *(draw_buffer+count) = atoi(token);
            //     count++;
                
            //     // Get next token
            //     token = strtok_r(NULL, ",", &saveptr);
            // }
            
            // free(buffer_copy);
        }
        else{
            sscanf( cmd ,"x1:%d,y1:%d,x2:%d,y2:%d,x3:%d,y3:%d,x4:%d,y4:%d", &draw_area[0].x, &draw_area[0].y, &draw_area[1].x, &draw_area[1].y, &draw_area[2].x, &draw_area[2].y, &draw_area[3].x, &draw_area[3].y);
            pr_info("x1:%d,y1:%d,x2:%d,y2:%d,x3:%d,y3:%d,x4:%d,y4:%d\n", draw_area[0].x, draw_area[0].y, draw_area[1].x, draw_area[1].y, draw_area[2].x, draw_area[2].y, draw_area[3].x, draw_area[3].y);
        }
    }

    return len;
}

static long tft_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int time = 100;
    struct tft_color color;
    
    /* Check if command is for our driver */
    if (_IOC_TYPE(cmd) != TFT_IOC_MAGIC) {
        return -ENOTTY;
    }
    
    switch (cmd) {
        case TFT_SET_TEXT_COLOR:
            if (copy_from_user(&color, (struct tft_color*)arg, sizeof(struct tft_color))) {
                return -EFAULT;
            }
            
            current_text_color = color;
            
            printk(KERN_INFO "TFT: Text color set to RGB(%d, %d, %d) u16(%d)\n", color.red, color.green, color.blue, TFT_STRUCT_TO_U16(current_text_color));
            printk(KERN_INFO "TFT: Text color set to RGB(%d, %d, %d) u16(%d)\n", current_text_color.red, current_text_color.green, current_text_color.blue, TFT_STRUCT_TO_U16(current_text_color));
            for(int i = 0; i < 25; i++)
                draw_buffer[i] = (u16)TFT_STRUCT_TO_U16(current_text_color);

            break;
            
        case TFT_GET_TEXT_COLOR:
            if (copy_to_user((struct tft_color*)arg, &current_text_color, sizeof(struct tft_color))) {
                return -EFAULT;
            }
            break;
            
        case TFT_SET_BG_COLOR:
            if (copy_from_user(&color, (struct tft_color*)arg, sizeof(struct tft_color))) {
                return -EFAULT;
            }
            
            current_bg_color = color;
            /* You would implement background color setting here */
            
            printk(KERN_INFO "TFT: Background color set to RGB(%d, %d, %d)\n", 
                   color.red, color.green, color.blue);
            break;
            
        case TFT_RESET_COLORS:
            current_text_color = (struct tft_color){0xFF, 0xFF, 0xFF};
            current_bg_color = (struct tft_color){0x00, 0x00, 0x00};
            
            printk(KERN_INFO "TFT: Colors reset to defaults\n");
            break;
            
            case TFT_SET_TIME_INTER:
            if (copy_from_user(&time, (int*)arg, sizeof(int))) {
                return -EFAULT;
            }
            time_interval = time;
            printk(KERN_INFO "TFT: Time interval set to %dms\n", time_interval);
            break;
                
        default:
            return -ENOTTY;
    }
    
    return ret;
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
    spi->mode &= ~SPI_CS_HIGH;
    spi->max_speed_hz = SPI_TFT_CLOCK_SPEED_HZ;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);

    if(ret < 0){
        dev_err(&spi->dev, "SPI setup failed: %d\n", ret);
        return ret;
    }

    spi->mode &= ~SPI_CS_HIGH;
    dev_info(&spi->dev, "SPI Mode: %d, Max Speed: %d Hz\n", spi->mode, spi->max_speed_hz);

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

    //Command sequence to init TFT
    if(!ra8875_init(data)){
        pr_err("Error on the init of the TFT screen\n");
        return -1;
    }

    // Create and start the kernel thread
    bounce_thread = kthread_run(text_bounce_func, (void*)data , "bounce_text_thread");
    if (IS_ERR(bounce_thread)) {
        printk(KERN_ERR "Failed to create kernel thread\n");
        return PTR_ERR(bounce_thread);
    }

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

    
    // Stop the kernel thread
    stop_thread = 1;
    if (bounce_thread) {
        kthread_stop(bounce_thread);
        bounce_thread = NULL;
    }
    
    ra8875_enable_display(data->spi, false);

    /* Set GPIO to safe state before removing */
    if (data->rst_pin) {
        gpiod_set_value(data->rst_pin, 0);
        msleep(100);
        gpiod_set_value(data->rst_pin, 1);
        dev_info(&spi->dev, "GPIO set to high before removal\n");
        /* GPIO needs to be freed manually */
        gpiod_put(data->rst_pin);
        data->rst_pin = NULL;
    }


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
