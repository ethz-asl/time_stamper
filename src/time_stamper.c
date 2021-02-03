#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sizes.h>

// Constants
/*****************************************/
#define DRV_NAME           "time_stamper"
#define N_BUFFER_ENTRIES   1024

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Mueller");
MODULE_DESCRIPTION("Raspberry Pi kernel module to timestamp GPIO events via IRQ");
MODULE_VERSION("0.01");

// Typedefs
/*****************************************/
struct TimestampRingBuffer {
    int head;
    int tail;
    struct {
        u64 id;
        u64 nsecs;
    } bufferentries[N_BUFFER_ENTRIES];
};

// function prototypes
/*****************************************/
static ssize_t ts_buffer_readout(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t ts_buffer_clear(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);

// Global variables
/*****************************************/
static struct kobject *ts_kobject;  
static struct TimestampRingBuffer ringbuffer;
// attributes for the sysfs communication, which functions are called by read or write access of the sysfs file
static struct kobj_attribute ts_buffer_attribute =__ATTR(ts_buffer, 0664, ts_buffer_readout, ts_buffer_clear);

// Main
/*****************************************/

/**
 * Returns the entries of the buffer (event timestamps) in string format
*/
static ssize_t ts_buffer_readout(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return 0;
}

/**
 * Clears the buffer
*/
static ssize_t ts_buffer_clear(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    ringbuffer.head = 0;
    ringbuffer.tail = ringbuffer.head;

    return 0;
}

/**
 * Init function of the LKM, is called at startup
*/
static int __init ts_init(void) 
{
    int err;

    /// create sysfs kernel object folder for the file to access in the file system under /sys/kernel/time_stamper/
    ts_kobject = kobject_create_and_add("time_stamper", kernel_kobj);
    if(!ts_kobject)
           return -ENOMEM;

    /// create the actual file for access the buffer functions (/sys/kernel/time_stamper/ts_buffer)
    err = sysfs_create_file(ts_kobject, &ts_buffer_attribute.attr);
    if (err) {
           printk(KERN_ALERT DRV_NAME " : failed to create the sysfs file for the buffer access\n");
           return err;
    }       

    /// init the timestamp ring-buffer
    ringbuffer.head = 0;
    ringbuffer.tail = ringbuffer.head;

    printk(KERN_INFO "GPIO IRQ Timestamper initialized and started\n");
    return 0;
}

/**
 * Exit function of the LKM, is called when the LKM ends
*/
static void __exit ts_exit(void) 
{
    /// remove the kobject and all its sub-files
    kobject_put(ts_kobject);

    printk(KERN_INFO "GPIO IRQ Timestamper ended\n");
}

module_init(ts_init);
module_exit(ts_exit);
