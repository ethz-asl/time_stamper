#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

// Constants
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Mueller");
MODULE_DESCRIPTION("Raspberry Pi kernel module to timestamp GPIO events via IRQ");
MODULE_VERSION("0.01");

/**
 * Init function of the LKM, is called at startup
*/
static int __init time_stamper_init(void) {
    printk(KERN_INFO "GPIO IRQ Timestamper started\n");
    return 0;
}

/**
 * Exit function of the LKM, is called when the LKM ends
*/
static void __exit time_stamper_exit(void) {
    printk(KERN_INFO "GPIO IRQ Timestamper started\n");
}

module_init(time_stamper_init);
module_exit(time_stamper_exit);
