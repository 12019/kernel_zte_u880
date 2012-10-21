
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	
#include <linux/fs.h>		
#include <linux/types.h>	
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <asm/system.h>		
#include <linux/uaccess.h>	


#include <mach/gpio.h>
#include "smscoreapi.h"

#include "smscharioctl.h"
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include <linux/spinlock.h>
#include <linux/wakelock.h>

#include <mach/pxa910_pm.h>


#define SMS_CHR_MAX_Q_LEN	15
#define SMSCHAR_NR_DEVS	        17 	

struct smschar_device_t {
	struct cdev cdev;	
	wait_queue_head_t waitq;	
	int cancel_waitq;
	spinlock_t lock;	
	int pending_count;
	struct list_head pending_data;	
	struct smscore_buffer_t *currentcb;
	int device_index;
	struct smscore_device_t *coredev;
	struct smscore_client_t *smsclient;
};

struct cmmb_lock {
	spinlock_t	lock;
	unsigned long	flags;
	int		dev_idx;
	int		count;
};


int smschar_major = 251;


int smschar_minor;  


module_param(smschar_major, int, S_IRUGO);
module_param(smschar_minor, int, S_IRUGO);

void set_ldo8(int enable);

struct smschar_device_t smschar_devices[SMSCHAR_NR_DEVS];
static int g_smschar_inuse =0 ;

static int g_pnp_status_changed = 1;


static struct class *smschr_dev_class;
static int g_has_suspended =0 ;
static struct device* sms_power_dev ;

int        sms_suspend_count  ;

static int        g_has_opened=0;
static int        g_has_opened_first=0;
static int resume_flag=0;

static int set_flag = 0;

static int wakelock_count = 0;

struct wake_lock cmmb_wakelock;

static struct cmmb_lock cmmb_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.count		= 0,
};

void cmmb_enable_lowpower(void)
{
      int boardid = pm860x_get_boardID();
 
 printk("cmmb_enable_lowpower\n");
	wakelock_count--;
	if (!wakelock_count)
		wake_unlock(&cmmb_wakelock);     
       
    	#ifdef CONFIG_PXA_U802
   	if(boardid == ZTE_HWVERSION2)
    	{
            set_ldo8(0); 
    	}
   	#endif
        
 printk("exit cmmb_enable_lowpower\n");
}

void cmmb_disable_lowpower(void)
{
       int boardid = pm860x_get_boardID();
 
 printk("cmmb_disable_lowpower\n");
	if (!wakelock_count)
		wake_lock(&cmmb_wakelock);
	wakelock_count++;

       
    	#ifdef CONFIG_PXA_U802
   	if(boardid == ZTE_HWVERSION2)
    	{
            set_ldo8(1); 
    	}
   	#endif
    
       
 printk("exit cmmb_disable_lowpower\n");
}

static void set_cmmb_constraint(void)
{
 printk("set_cmmb_constraint\n");
	
	if (set_flag)
		return;
	spin_lock_irqsave(&cmmb_lock.lock, cmmb_lock.flags);
	cmmb_disable_lowpower();
	spin_unlock_irqrestore(&cmmb_lock.lock, cmmb_lock.flags);
	set_flag = 1;
}

static void unset_cmmb_constraint(void)
{
 printk("unset_cmmb_constraint\n");
        if (!set_flag)
		return;
	spin_lock_irqsave(&cmmb_lock.lock, cmmb_lock.flags);
	cmmb_enable_lowpower();
	spin_unlock_irqrestore(&cmmb_lock.lock, cmmb_lock.flags);
	set_flag = 0;
}


static void smschar_unregister_client(struct smschar_device_t *dev)
{
	unsigned long flags;

	sms_info("entering... smschar_unregister_client....\n");
	if (dev->coredev && dev->smsclient) {
		dev->cancel_waitq = 1;
		wake_up_interruptible(&dev->waitq);

		spin_lock_irqsave(&dev->lock, flags);

		while (!list_empty(&dev->pending_data)) {
			struct smscore_buffer_t *cb =
			    (struct smscore_buffer_t *)dev->pending_data.next;
			list_del(&cb->entry);

			smscore_putbuffer(dev->coredev, cb);
			dev->pending_count--;
		}

		if (dev->currentcb) {
			smscore_putbuffer(dev->coredev, dev->currentcb);
			dev->currentcb = NULL;
			dev->pending_count--;
		}

		smscore_unregister_client(dev->smsclient);
		dev->smsclient = NULL;

		spin_unlock_irqrestore(&dev->lock, flags);
	}
}


static int smschar_onresponse(void *context, struct smscore_buffer_t *cb)
{
	struct smschar_device_t *dev = context;
	unsigned long flags;

	if (!dev) {
		sms_err("recieved bad dev pointer\n");
		return -EFAULT;
	}
	spin_lock_irqsave(&dev->lock, flags);

	if (dev->pending_count > SMS_CHR_MAX_Q_LEN) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}

	dev->pending_count++;
	
	if (dev->device_index) {
		cb->size -= sizeof(struct SmsMsgHdr_ST);
		cb->offset += sizeof(struct SmsMsgHdr_ST);
	}

	list_add_tail(&cb->entry, &dev->pending_data);
	spin_unlock_irqrestore(&dev->lock, flags);

	if (waitqueue_active(&dev->waitq))
		wake_up_interruptible(&dev->waitq);

	return 0;
}


static void smschar_onremove(void *context)
{
	struct smschar_device_t *dev = (struct smschar_device_t *)context;

	smschar_unregister_client(dev);
	dev->coredev = NULL;
}


static int smschar_open(struct inode *inode, struct file *file)
{
	struct smschar_device_t *dev = container_of(inode->i_cdev,
						    struct smschar_device_t,
						    cdev);
	int rc = -ENODEV;

      
        printk("smschar_open\n");

        set_cmmb_constraint();

	if (dev->coredev) {
		struct smsclient_params_t params;
      #if 1

		if(g_has_opened_first==0 && dev->device_index==0)
		{
		 smsspi_poweron();
		 g_has_opened_first=1;
		 printk("open first********\n");

	       }
	       else if(dev->device_index!=0)
                g_has_opened_first=0;
      
      #endif           		

		params.initial_id = dev->device_index ?
		    dev->device_index : SMS_HOST_LIB;
		params.data_type = dev->device_index ? MSG_SMS_DAB_CHANNEL : 0;
		params.onresponse_handler = smschar_onresponse;
		params.onremove_handler = smschar_onremove;
		params.context = dev;

		rc = smscore_register_client(dev->coredev, &params,
					     &dev->smsclient);
		if (!rc)
			file->private_data = dev;
		dev->cancel_waitq = 0;
		g_pnp_status_changed = 1;
	        g_has_opened++;	
	
	}

       
	if (rc)
		sms_err(" exiting, rc %d\n", rc);

	return rc;
}


static int smschar_release(struct inode *inode, struct file *file)
{
	struct smschar_device_t *dev = file->private_data;

printk("smschar_release\n");
	smschar_unregister_client(file->private_data);
    #if 1

	if(!(--g_has_opened)&& (g_has_opened_first==0))
	{
	     smscore_reset_device_drvs(dev->coredev);
             smsspi_off();
             g_has_opened_first=0;
	     printk("release at the end******\n");
        }
   
    #endif
	sms_info("exiting\n");

        unset_cmmb_constraint();

	return 0;
}


static ssize_t smschar_read(struct file *file, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct smschar_device_t *dev = file->private_data;
	unsigned long flags;
	int rc, copied = 0;

	if (!buf) {
		sms_err("Bad pointer recieved from user.\n");
		return -EFAULT;
	}
	if (!dev->coredev || !dev->smsclient||g_has_suspended) {
		sms_err("no client\n");
		return -ENODEV;
	}
	rc = wait_event_interruptible(dev->waitq,
				      !list_empty(&dev->pending_data)
				      || (dev->cancel_waitq));
	if (rc < 0) {
		sms_err("wait_event_interruptible error %d\n", rc);
		return rc;
	}
	if (dev->cancel_waitq)
		return 0;
	if (!dev->smsclient) {
		sms_err("no client\n");
		return -ENODEV;
	}
	spin_lock_irqsave(&dev->lock, flags);

	while (!list_empty(&dev->pending_data) && (copied < count)) {
		struct smscore_buffer_t *cb =
		    (struct smscore_buffer_t *)dev->pending_data.next;
		int actual_size = min(((int)count - copied), cb->size);
		if (copy_to_user(&buf[copied], &((char *)cb->p)[cb->offset],
				 actual_size)) {
			sms_err("copy_to_user failed\n");
			spin_unlock_irqrestore(&dev->lock, flags);
			return -EFAULT;
		}
		copied += actual_size;
		cb->offset += actual_size;
		cb->size -= actual_size;

		if (!cb->size) {
			list_del(&cb->entry);
			smscore_putbuffer(dev->coredev, cb);
			dev->pending_count--;
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return copied;
}


static ssize_t smschar_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct smschar_device_t *dev;
	void *buffer;

         
	if (file == NULL) {
		sms_err("file is NULL\n");
		return EINVAL;
	}

	if (file->private_data == NULL) {
		sms_err("file->private_data is NULL\n");
		return -EINVAL;
	}

	dev = file->private_data;
	if (!dev->smsclient||g_has_suspended) {
		sms_err("no client\n");
		return -ENODEV;
	}

	buffer = kmalloc(ALIGN(count, SMS_ALLOC_ALIGNMENT) + SMS_DMA_ALIGNMENT,
			 GFP_KERNEL | GFP_DMA);
	if (buffer) {
		void *msg_buffer = (void *)SMS_ALIGN_ADDRESS(buffer);

		if (!copy_from_user(msg_buffer, buf, count))
			smsclient_sendrequest(dev->smsclient,
					      msg_buffer, count);
		else
			count = 0;

		kfree(buffer);
	}

	return count;
}

static int smschar_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct smschar_device_t *dev = file->private_data;
	return smscore_map_common_buffer(dev->coredev, vma);
}


static int smschar_wait_get_buffer(struct smschar_device_t *dev,
				   struct smschar_buffer_t *touser)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->currentcb) {
		smscore_putbuffer(dev->coredev, dev->currentcb);
		dev->currentcb = NULL;
		dev->pending_count--;
	}

	spin_unlock_irqrestore(&dev->lock, flags);


	memset(touser, 0, sizeof(struct smschar_buffer_t));

	rc = wait_event_interruptible(dev->waitq,
				      !list_empty(&dev->pending_data)
				      || (dev->cancel_waitq));
	if (rc < 0) {
		sms_err("wait_event_interruptible error, rc=%d\n", rc);
		return rc;
	}
	if (dev->cancel_waitq) {
		touser->offset = 0;
		touser->size = 0;
		return 0;
	}
	if (!dev->smsclient) {
		sms_err("no client\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&dev->lock, flags);

	if (!list_empty(&dev->pending_data)) {
		struct smscore_buffer_t *cb =
		    (struct smscore_buffer_t *)dev->pending_data.next;
		touser->offset = cb->offset_in_common + cb->offset;
		touser->size = cb->size;

		list_del(&cb->entry);

		dev->currentcb = cb;
	} else {
		touser->offset = 0;
		touser->size = 0;
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}


static unsigned int smschar_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct smschar_device_t *dev;
	int mask = 0;

	if (file == NULL) {
		sms_err("file is NULL\n");
		return EINVAL;
	}

	if (file->private_data == NULL) {
		sms_err("file->private_data is NULL\n");
		return -EINVAL;
	}

	dev = file->private_data;

	if (list_empty(&dev->pending_data)) {
		sms_info("No data is ready, waiting for data recieve.\n");
		poll_wait(file, &dev->waitq, wait);
	}

	if (!list_empty(&dev->pending_data))
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static int smschar_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct smschar_device_t *dev = file->private_data;
	void __user *up = (void __user *)arg;

	if (!dev->coredev || !dev->smsclient||g_has_suspended) {
		sms_err("no client\n");
		return -ENODEV;
	}
    

	switch (cmd) {
	case SMSCHAR_STARTUP:
                smsspi_poweron();
		return 0;
	case SMSCHAR_SET_DEVICE_MODE:
		return smscore_set_device_mode(dev->coredev, (int)arg);

	case SMSCHAR_GET_DEVICE_MODE:
		{
			if (put_user(smscore_get_device_mode(dev->coredev),
				     (int *)up))
				return -EFAULT;
			break;
		}
	case SMSCHAR_IS_DEVICE_PNP_EVENT:
		{
                       printk("pnp event not supported\n") ;
#if 0
			sms_info("Waiting for PnP event.\n");
			wait_event_interruptible(g_pnp_event,
						 !g_pnp_status_changed);
			g_pnp_status_changed = 0;
			sms_info("PnP Event %d.\n", g_smschar_inuse);
			if (put_user(g_smschar_inuse, (int *)up))
				return -EFAULT;
#endif 
			break;
		}
	case SMSCHAR_GET_BUFFER_SIZE:
		{

			if (put_user
			    (smscore_get_common_buffer_size(dev->coredev),
			     (int *)up))
				return -EFAULT;

			break;
		}

	case SMSCHAR_WAIT_GET_BUFFER:
		{
			struct smschar_buffer_t touser;
			int rc;

			rc = smschar_wait_get_buffer(dev, &touser);
			if (rc < 0)
				return rc;

			if (copy_to_user(up, &touser,
					 sizeof(struct smschar_buffer_t)))
				return -EFAULT;

			break;
		}
	case SMSCHAR_CANCEL_WAIT_BUFFER:
		{
			dev->cancel_waitq = 1;
			wake_up_interruptible(&dev->waitq);
			break;
		}
	case SMSCHAR_GET_FW_FILE_NAME:
		{
			if (!up)
				return -EINVAL;
			return smscore_get_fw_filename(dev->coredev,
				       ((struct
					 smschar_get_fw_filename_ioctl_t
					 *)up)->mode,
				       ((struct
					 smschar_get_fw_filename_ioctl_t
					 *)up)->filename);
		}
	case SMSCHAR_SEND_FW_FILE:
		{
			if (!up)
				return -EINVAL;
			return smscore_send_fw_file(dev->coredev,
					((struct
					smschar_send_fw_file_ioctl_t
					*)up)->fw_buf,
					((struct
					smschar_send_fw_file_ioctl_t
					*)up)->fw_size);
		}

	case  SMSCHAR_GET_RESUME_FLAG:
		 if (copy_to_user(up, &resume_flag, sizeof(int)))
			  return -EFAULT;
		 else
		 return 0;
    		  
	case  SMSCHAR_SET_RESUME_FLAG:
		 if (copy_from_user(&resume_flag,up, sizeof(int)))
		 	return -EFAULT;
		 else
	     return 0;

		  
	case  SMSCHAR_RESET_DEVICE_DRVS:
              smsspi_off();
	     return  smscore_reset_device_drvs (dev->coredev);

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}


struct file_operations smschar_fops = {
	.owner = THIS_MODULE,
	.read = smschar_read,
	.write = smschar_write,
	.open = smschar_open,
	.release = smschar_release,
	.mmap = smschar_mmap,
	.poll = smschar_poll,
	.ioctl = smschar_ioctl,
};

static int smschar_setup_cdev(struct smschar_device_t *dev, int index)
{
	
	int rc, devno = MKDEV(smschar_major, smschar_minor + index);

	cdev_init(&dev->cdev, &smschar_fops);

	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &smschar_fops;

	kobject_set_name(&dev->cdev.kobj, "Siano_sms%d", index);
	rc = cdev_add(&dev->cdev, devno, 1);
	
	if (!index)
	{
		device_create(smschr_dev_class, NULL, devno, NULL, "mdtvctrl");
	}
	else
	{	
		device_create(smschr_dev_class, NULL, devno, NULL, "mdtv%d", index);
	}
	
	sms_info("exiting %p %d, rc %d", dev, index, rc);

	return rc;
}


static int smschar_hotplug(struct smscore_device_t *coredev,
			   struct device *device, int arrival)
{
	int rc = 0, i;

	sms_info("entering %d\n", arrival);

	g_pnp_status_changed = 1;
	if (arrival) {
		
		if (!g_smschar_inuse) {
			
			memset(smschar_devices, 0, SMSCHAR_NR_DEVS *
			       sizeof(struct smschar_device_t));

			
			for (i = 0; i < SMSCHAR_NR_DEVS; i++) {
				sms_info("create device %d", i);
				smschar_setup_cdev(&smschar_devices[i], i);
				INIT_LIST_HEAD(&smschar_devices[i].
					       pending_data);
				spin_lock_init(&smschar_devices[i].lock);
				init_waitqueue_head(&smschar_devices[i].waitq);

				smschar_devices[i].coredev = coredev;
				smschar_devices[i].device_index = i;
			}
			g_smschar_inuse = 1;

		}
	} else {
		
		if (g_smschar_inuse) {
			
			for (i = 0; i < SMSCHAR_NR_DEVS; i++) {
				cdev_del(&smschar_devices[i].cdev);
				sms_info("remove device %d\n", i);
			}

			g_smschar_inuse = 0;

		}
	}

	sms_info("exiting, rc %d\n", rc);

	return rc;		
}

void smschar_reset_device(void)
{
    int i;
    printk(KERN_EMERG "SMS1180:in smschar_reset_device\n") ;
    for(i=0;i< SMSCHAR_NR_DEVS;i++)
    {
        smschar_devices[i].cancel_waitq = 1;
        wake_up_interruptible(&smschar_devices[i].waitq) ;
        smschar_unregister_client(&smschar_devices[i]) ;
    }
}
void smschar_set_suspend(int suspend_on)
{
    printk(KERN_EMERG "SMS1180 : suspend_on = %d\n",suspend_on) ;
    if(suspend_on) 
       g_has_suspended = 1;
    else 
       g_has_suspended = 0;
}

EXPORT_SYMBOL(smschar_reset_device) ;
EXPORT_SYMBOL(smschar_set_suspend) ;

static ssize_t
sms_suspend_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d",sms_suspend_count) ;
}
static ssize_t
sms_suspend_state_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count) 
{
    sms_suspend_count =0 ;
    return count ;
}

static DEVICE_ATTR(suspend,S_IRUGO|S_IWUGO,sms_suspend_state_show,sms_suspend_state_store);

#ifdef CONFIG_PM
#ifdef CONFIG_ANDROID_POWER
void smsspi_android_suspend_handler(android_early_suspend_t *h)
{
}

void smsspi_android_resume_handler(android_early_suspend_t *h)
{
	  int value;
	  if(g_has_opened)
	  {
	  resume_flag=1;
	  }
	  else
	  resume_flag=0;
}
static android_early_suspend_t smsspi_android_suspend = {
	.level = 5,
	.suspend = smsspi_android_suspend_handler,
	.resume = smsspi_android_resume_handler,
};
#endif
#endif 
int smschar_register(void)
{
	dev_t devno = MKDEV(smschar_major, smschar_minor);
	int rc;

	sms_info("registering device major=%d minor=%d\n", smschar_major,
		 smschar_minor);

        wake_lock_init(&cmmb_wakelock, WAKE_LOCK_SUSPEND, "cmmb_wakeup");

	if (smschar_major) {
		rc = register_chrdev_region(devno, SMSCHAR_NR_DEVS, "smschar");
	} else {
		rc = alloc_chrdev_region(&devno, smschar_minor,
					 SMSCHAR_NR_DEVS, "smschar");
		smschar_major = MAJOR(devno);
	}

	if (rc < 0) {
		sms_warn("smschar: can't get major %d\n", smschar_major);
		return rc;
	}


	smschr_dev_class = class_create(THIS_MODULE, "smsmdtv");
	if(IS_ERR(smschr_dev_class)){
		sms_err("Could not create sms char device class\n");
		return -1;
	}
        sms_power_dev = device_create(smschr_dev_class,NULL,0,"%s","power_state") ;
        if(sms_power_dev)
        {
           rc = device_create_file(sms_power_dev, &dev_attr_suspend) ;
        }
	
	return smscore_register_hotplug(smschar_hotplug);
}

void smschar_unregister(void)
{
	dev_t devno = MKDEV(smschar_major, smschar_minor);
	
	int i;
	for( i = 0; i < SMSCHAR_NR_DEVS; i++)
		device_destroy(smschr_dev_class, MKDEV(smschar_major, i));
	class_destroy(smschr_dev_class);

	unregister_chrdev_region(devno, SMSCHAR_NR_DEVS);
	smscore_unregister_hotplug(smschar_hotplug);
	
	sms_info("unregistered\n");
}
