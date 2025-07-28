#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DEVICE_NAME "embedded_debugger"
#define CLASS_NAME "embedded_debug"
#define I2C_SLAVE_ADDR 0x48

struct embedded_device {
    struct cdev cdev;
    struct device *device;
    struct i2c_client *i2c_client;
    dev_t dev_num;
    struct class *device_class;
    void __iomem *reg_base;
    int irq;
    wait_queue_head_t read_queue;
    spinlock_t lock;
    u32 sensor_data[16];
    int data_ready;
};

static struct embedded_device *embed_dev;

// I2C read/write functions
static int i2c_read_sensor(struct i2c_client *client, u8 reg, u8 *data)
{
    struct i2c_msg msgs[2];
    int ret;
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;
    
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = data;
    
    ret = i2c_transfer(client->adapter, msgs, 2);
    return (ret == 2) ? 0 : -EIO;
}

static int i2c_write_sensor(struct i2c_client *client, u8 reg, u8 data)
{
    u8 buf[2] = {reg, data};
    struct i2c_msg msg = {
        .addr = client->addr,
        .flags = 0,
        .len = 2,
        .buf = buf,
    };
    
    return (i2c_transfer(client->adapter, &msg, 1) == 1) ? 0 : -EIO;
}

// Interrupt handler
static irqreturn_t embedded_irq_handler(int irq, void *dev_id)
{
    struct embedded_device *dev = (struct embedded_device *)dev_id;
    unsigned long flags;
    u8 sensor_val;
    
    // Read sensor data via I2C
    if (i2c_read_sensor(dev->i2c_client, 0x00, &sensor_val) == 0) {
        spin_lock_irqsave(&dev->lock, flags);
        dev->sensor_data[dev->data_ready % 16] = sensor_val;
        dev->data_ready++;
        spin_unlock_irqrestore(&dev->lock, flags);
        
        wake_up_interruptible(&dev->read_queue);
    }
    
    return IRQ_HANDLED;
}

// Device file operations
static int embedded_open(struct inode *inode, struct file *file)
{
    struct embedded_device *dev = container_of(inode->i_cdev, 
                                               struct embedded_device, cdev);
    file->private_data = dev;
    
    printk(KERN_INFO "Embedded debugger device opened\n");
    return 0;
}

static int embedded_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "Embedded debugger device closed\n");
    return 0;
}

static ssize_t embedded_read(struct file *file, char __user *user_buf, 
                           size_t count, loff_t *pos)
{
    struct embedded_device *dev = file->private_data;
    unsigned long flags;
    u32 data_to_copy;
    int bytes_to_copy = min(count, sizeof(u32));
    
    // Wait for data if none available
    if (wait_event_interruptible(dev->read_queue, dev->data_ready > 0))
        return -ERESTARTSYS;
    
    spin_lock_irqsave(&dev->lock, flags);
    if (dev->data_ready > 0) {
        data_to_copy = dev->sensor_data[(dev->data_ready - 1) % 16];
        dev->data_ready--;
    } else {
        spin_unlock_irqrestore(&dev->lock, flags);
        return 0;
    }
    spin_unlock_irqrestore(&dev->lock, flags);
    
    if (copy_to_user(user_buf, &data_to_copy, bytes_to_copy))
        return -EFAULT;
    
    return bytes_to_copy;
}

static ssize_t embedded_write(struct file *file, const char __user *user_buf,
                            size_t count, loff_t *pos)
{
    struct embedded_device *dev = file->private_data;
    u8 reg, data;
    
    if (count < 2)
        return -EINVAL;
    
    if (copy_from_user(&reg, user_buf, 1) || 
        copy_from_user(&data, user_buf + 1, 1))
        return -EFAULT;
    
    if (i2c_write_sensor(dev->i2c_client, reg, data))
        return -EIO;
    
    return count;
}

static const struct file_operations embedded_fops = {
    .owner = THIS_MODULE,
    .open = embedded_open,
    .release = embedded_release,
    .read = embedded_read,
    .write = embedded_write,
};

// Platform driver probe
static int embedded_probe(struct platform_device *pdev)
{
    struct resource *res;
    int ret;
    
    embed_dev = devm_kzalloc(&pdev->dev, sizeof(*embed_dev), GFP_KERNEL);
    if (!embed_dev)
        return -ENOMEM;
    
    // Get memory resource
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    embed_dev->reg_base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(embed_dev->reg_base))
        return PTR_ERR(embed_dev->reg_base);
    
    // Get IRQ
    embed_dev->irq = platform_get_irq(pdev, 0);
    if (embed_dev->irq < 0)
        return embed_dev->irq;
    
    // Initialize wait queue and spinlock
    init_waitqueue_head(&embed_dev->read_queue);
    spin_lock_init(&embed_dev->lock);
    
    // Allocate character device
    ret = alloc_chrdev_region(&embed_dev->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        printk(KERN_ERR "Failed to allocate char device region\n");
        return ret;
    }
    
    // Initialize and add character device
    cdev_init(&embed_dev->cdev, &embedded_fops);
    embed_dev->cdev.owner = THIS_MODULE;
    
    ret = cdev_add(&embed_dev->cdev, embed_dev->dev_num, 1);
    if (ret < 0) {
        unregister_chrdev_region(embed_dev->dev_num, 1);
        return ret;
    }
    
    // Create device class
    embed_dev->device_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(embed_dev->device_class)) {
        cdev_del(&embed_dev->cdev);
        unregister_chrdev_region(embed_dev->dev_num, 1);
        return PTR_ERR(embed_dev->device_class);
    }
    
    // Create device
    embed_dev->device = device_create(embed_dev->device_class, NULL, 
                                     embed_dev->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(embed_dev->device)) {
        class_destroy(embed_dev->device_class);
        cdev_del(&embed_dev->cdev);
        unregister_chrdev_region(embed_dev->dev_num, 1);
        return PTR_ERR(embed_dev->device);
    }
    
    // Request IRQ
    ret = request_irq(embed_dev->irq, embedded_irq_handler, 
                     IRQF_TRIGGER_RISING, DEVICE_NAME, embed_dev);
    if (ret) {
        device_destroy(embed_dev->device_class, embed_dev->dev_num);
        class_destroy(embed_dev->device_class);
        cdev_del(&embed_dev->cdev);
        unregister_chrdev_region(embed_dev->dev_num, 1);
        return ret;
    }
    
    platform_set_drvdata(pdev, embed_dev);
    
    printk(KERN_INFO "Embedded debugger driver loaded successfully\n");
    printk(KERN_INFO "Major number: %d\n", MAJOR(embed_dev->dev_num));
    
    return 0;
}

static int embedded_remove(struct platform_device *pdev)
{
    struct embedded_device *dev = platform_get_drvdata(pdev);
    
    free_irq(dev->irq, dev);
    device_destroy(dev->device_class, dev->dev_num);
    class_destroy(dev->device_class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->dev_num, 1);
    
    printk(KERN_INFO "Embedded debugger driver removed\n");
    return 0;
}

static const struct of_device_id embedded_of_match[] = {
    { .compatible = "ecen449,embedded-debugger", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, embedded_of_match);

static struct platform_driver embedded_driver = {
    .probe = embedded_probe,
    .remove = embedded_remove,
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = embedded_of_match,
    },
};

module_platform_driver(embedded_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ECEN 449 Student");
MODULE_DESCRIPTION("Embedded Systems Debugger with I2C and Interrupt Support");
MODULE_VERSION("1.0");
