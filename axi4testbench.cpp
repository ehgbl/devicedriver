module axi4_testbench;
    
    // Clock and reset
    logic clk;
    logic rst_n;
    
    // AXI4 Master Interface
    logic [31:0] m_axi_awaddr;
    logic [7:0]  m_axi_awlen;
    logic [2:0]  m_axi_awsize;
    logic [1:0]  m_axi_awburst;
    logic        m_axi_awvalid;
    logic        m_axi_awready;
    
    logic [31:0] m_axi_wdata;
    logic [3:0]  m_axi_wstrb;
    logic        m_axi_wlast;
    logic        m_axi_wvalid;
    logic        m_axi_wready;
    
    logic [1:0]  m_axi_bresp;
    logic        m_axi_bvalid;
    logic        m_axi_bready;
    
    logic [31:0] m_axi_araddr;
    logic [7:0]  m_axi_arlen;
    logic [2:0]  m_axi_arsize;
    logic [1:0]  m_axi_arburst;
    logic        m_axi_arvalid;
    logic        m_axi_arready;
    
    logic [31:0] m_axi_rdata;
    logic [1:0]  m_axi_rresp;
    logic        m_axi_rlast;
    logic        m_axi_rvalid;
    logic        m_axi_rready;
    
    // Coverage and assertions
    covergroup axi4_coverage @(posedge clk);
        write_burst: coverpoint m_axi_awlen {
            bins single = {0};
            bins burst_4 = {3};
            bins burst_8 = {7};
            bins burst_16 = {15};
        }
        
        read_burst: coverpoint m_axi_arlen {
            bins single = {0};
            bins burst_4 = {3};
            bins burst_8 = {7};
            bins burst_16 = {15};
        }
        
        write_response: coverpoint m_axi_bresp {
            bins okay = {2'b00};
            bins exokay = {2'b01};
            bins slverr = {2'b10};
            bins decerr = {2'b11};
        }
    endgroup
    
    axi4_coverage cov_inst = new();
    
    // Assertions
    property axi4_handshake_valid_ready(valid, ready);
        @(posedge clk) disable iff (!rst_n)
        valid && !ready |=> valid;
    endproperty
    
    assert property (axi4_handshake_valid_ready(m_axi_awvalid, m_axi_awready))
        else $error("AXI4 AW handshake violation");
    
    assert property (axi4_handshake_valid_ready(m_axi_wvalid, m_axi_wready))
        else $error("AXI4 W handshake violation");
    
    assert property (axi4_handshake_valid_ready(m_axi_arvalid, m_axi_arready))
        else $error("AXI4 AR handshake violation");
    
    // Clock generation
    always #5 clk = ~clk;
    
    // Test sequence
    initial begin
        clk = 0;
        rst_n = 0;
        
        // Initialize signals
        m_axi_awaddr = 0;
        m_axi_awlen = 0;
        m_axi_awsize = 3'b010; // 4 bytes
        m_axi_awburst = 2'b01; // INCR
        m_axi_awvalid = 0;
        
        m_axi_wdata = 0;
        m_axi_wstrb = 4'hF;
        m_axi_wlast = 0;
        m_axi_wvalid = 0;
        
        m_axi_bready = 1;
        
        m_axi_araddr = 0;
        m_axi_arlen = 0;
        m_axi_arsize = 3'b010;
        m_axi_arburst = 2'b01;
        m_axi_arvalid = 0;
        
        m_axi_rready = 1;
        
        // Reset sequence
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);
        
        // Test single write
        axi4_write(32'h1000, 32'hDEADBEEF, 0);
        
        // Test burst write (4 beats)
        axi4_burst_write(32'h2000, 3, {32'h12345678, 32'h9ABCDEF0, 32'hFEDCBA98, 32'h76543210});
        
        // Test single read
        axi4_read(32'h1000, 0);
        
        // Test burst read (4 beats)
        axi4_burst_read(32'h2000, 3);
        
        // Error injection test
        axi4_write(32'hFFFF_FFFF, 32'h12345678, 0); // Invalid address
        
        repeat(100) @(posedge clk);
        
        $display("Functional Coverage: %0.2f%%", cov_inst.get_coverage());
        $finish;
    end
    
    // Write transaction task
    task axi4_write(input [31:0] addr, input [31:0] data, input [7:0] len);
        fork
            begin
                // Address channel
                m_axi_awaddr = addr;
                m_axi_awlen = len;
                m_axi_awvalid = 1;
                @(posedge clk iff m_axi_awready);
                m_axi_awvalid = 0;
            end
            begin
                // Data channel
                repeat(len + 1) begin
                    m_axi_wdata = data;
                    m_axi_wvalid = 1;
                    m_axi_wlast = (len == 0);
                    @(posedge clk iff m_axi_wready);
                    data = data + 1; // Increment for burst
                    len = len - 1;
                end
                m_axi_wvalid = 0;
                m_axi_wlast = 0;
            end
        join
        
        // Wait for response
        @(posedge clk iff m_axi_bvalid);
        $display("Write response: %0h", m_axi_bresp);
    endtask
    
    // Burst write task
    task axi4_burst_write(input [31:0] addr, input [7:0] len, input [31:0] data_array[]);
        fork
            begin
                m_axi_awaddr = addr;
                m_axi_awlen = len;
                m_axi_awvalid = 1;
                @(posedge clk iff m_axi_awready);
                m_axi_awvalid = 0;
            end
            begin
                for(int i = 0; i <= len; i++) begin
                    m_axi_wdata = data_array[i];
                    m_axi_wvalid = 1;
                    m_axi_wlast = (i == len);
                    @(posedge clk iff m_axi_wready);
                end
                m_axi_wvalid = 0;
                m_axi_wlast = 0;
            end
        join
        
        @(posedge clk iff m_axi_bvalid);
        $display("Burst write response: %0h", m_axi_bresp);
    endtask
    
    // Read transaction task
    task axi4_read(input [31:0] addr, input [7:0] len);
        m_axi_araddr = addr;
        m_axi_arlen = len;
        m_axi_arvalid = 1;
        @(posedge clk iff m_axi_arready);
        m_axi_arvalid = 0;
        
        repeat(len + 1) begin
            @(posedge clk iff m_axi_rvalid);
            $display("Read data: %0h, resp: %0h, last: %0b", 
                     m_axi_rdata, m_axi_rresp, m_axi_rlast);
        end
    endtask
    
    // Burst read task
    task axi4_burst_read(input [31:0] addr, input [7:0] len);
        m_axi_araddr = addr;
        m_axi_arlen = len;
        m_axi_arvalid = 1;
        @(posedge clk iff m_axi_arready);
        m_axi_arvalid = 0;
        
        for(int i = 0; i <= len; i++) begin
            @(posedge clk iff m_axi_rvalid);
            $display("Burst read[%0d]: %0h, resp: %0h, last: %0b", 
                     i, m_axi_rdata, m_axi_rresp, m_axi_rlast);
        end
    endtask
    
endmodule

//=================================================================
// C++ Device Driver Framework for I2C Communication
//=================================================================

/* 
   Device Driver C++ Code (complementary to Verilog testbench)
   File: i2c_device_driver.cpp
*/

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
