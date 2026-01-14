#ifndef _MUDULE_TYPE_H_
#define _MUDULE_TYPE_H_
#define sprintf_s(str, size, ...) sprintf(str, __VA_ARGS__)

#define RSMC_ERROR (-1)
#define EOK        (0)
enum module_type {
	MODULE_TYPE_KNL = 0,
	MODULE_TYPE_CTRL,
	MODULE_TYPE_STACK,
	MODULE_TYPE_FAST_CTRL,
	MODULE_TYPE_MAX
};

#define cetclog_info(fmt, ...) \
    do { \
        printk(KERN_INFO "INFO(%s:%d) " fmt "\n", \
            __FILE__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#define cetclog_err(fmt, ...) \
    do { \
        printk(KERN_NOTICE "INFO(%s:%d) " fmt "\n", \
            __FILE__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#endif

