/******************************************************************************
 * hid-asusmt.c  --  Driver for Multi-Touch USB Touchscreens
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/uaccess.h>

#define DRIVER_VERSION "v0.4"
#define DRIVER_AUTHOR "Gavin Guo, mimi0213kimo@gmail.com"
#define DRIVER_DESC "Asus USB Multi-Touch Touch Panel driver"
#define DRIVER_LICENSE "GPL"


MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

#define USB_VENDOR_ID_ASUS 0x0486
#define USB_DEVICE_ID_ASUS 0x0185
#define VAIO_RDESC_CONSTANT 0x0001


#define ASUS_MT_STRING "asus_mt"
#define MAXQUEUE 100
#define DATANUM 4
u8 axies_data[DATANUM];
static int asus_mt_major;
static struct class *asus_mt_class;
static int head, tail;
static int ready;

struct data_queue {
	u32 axies_data[DATANUM];
};

struct data_queue queue[MAXQUEUE];

static bool swap_xy;
module_param(swap_xy, bool, 0644);
MODULE_PARM_DESC(swap_xy, "If set X and Y axes are swapped.");

struct asus_mt_usb {
	unsigned char *data;
	unsigned int data_size;
	struct	input_dev *input1, *input2;
	struct asmt_device_info *type;
	struct usb_device *udev;
	dma_addr_t data_dma;
	struct urb *urb;
	struct usb_interface *interface;
	char name[128];
	char phys0[64];
	char phys1[64];
	int x1, y1;
	int x2, y2;
	int tag1, tag2;
	int touch1, touch2, press;
	int p2;
	char input1_map_tag, input2_map_tag;
	char tag_array[5];
};

struct asmt_device_info {
	int min_xc, max_xc;
	int min_yc, max_yc;
	int min_press, max_press;
	int rept_size;

	void (*process_pkt)(struct asus_mt_usb *asmt,
			     unsigned char *pkt, int len);

	/*
	 * used to get the packet len. possible return values:
	 * > 0: packet len
	 * = 0: skip one byte
	 * < 0: -return value more bytes needed
	 */
	int (*get_pkt_len)(unsigned char *pkt, int len);

	int (*read_data)(struct asus_mt_usb *asmt, unsigned char *pkt);
	int (*init)(struct asus_mt_usb *asmt);
};

static int insert_queue(struct asus_mt_usb *asmt)
{
	int i;
	struct device *dev = &asmt->interface->dev;

	if ((head + 1) == tail) {
		dev_dbg(dev, "%s: data_queue is full, some data are losed\n",
			__func__);
		return -1;
	} else {
		head = (head + 1) % MAXQUEUE;
		for (i = 0; i < DATANUM; i++)
			queue[head].axies_data[i] = axies_data[i];
	}

	return 0;
}

static int delete_queue(unsigned long arg)
{
	if (head == tail)
		goto empty;
	else {
		tail = (tail + 1) % MAXQUEUE;
		if (copy_to_user((unsigned int *)arg, queue[tail].axies_data,
						sizeof(u8) * DATANUM)) {
			printk("error copy_to_user\n");
			return -EFAULT;
		}
		return 0;
	}
empty:
	printk("New data have not been coming yet..\n");
	return -1;
}

static long asus_mt_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	ready = 0;
	printk("asus_mt...ioctl\n");

	switch (cmd) {
	case 0:
		printk("delete_queue\n");
		delete_queue(arg);
		break;
	case 1:
		printk("query if queue has data\n");
		if (head == tail)
			*(unsigned int *)arg = (unsigned int)0;
		else
			*(unsigned int *)arg = (unsigned int)1;
		break;
	}
	return 0;
}

static int asus_mt_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int asus_mt_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations asus_mt_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asus_mt_ioctl,
	.open = asus_mt_open,
	.release = asus_mt_release,
};

static int asus_read_data(struct asus_mt_usb *dev, unsigned char *pkt)
{

	axies_data[0] = pkt[4];
	axies_data[1] = pkt[5] & 0x0F;
	axies_data[2] = pkt[6];
	axies_data[3] = pkt[7] & 0x0F;
	insert_queue(dev);

	dev->x1 = ((pkt[5] & 0x0F) << 8) | (pkt[4] & 0xFF);
	dev->y1 = ((pkt[7] & 0x0F) << 8) | (pkt[6] & 0xFF);
	dev->touch1 = pkt[1] & 0x03;
	dev->tag1 = pkt[3];

	dev->p2 = pkt[15] & 0x02;

	if (dev->p2) {
		dev->x2 = ((pkt[12] & 0x0F) << 8) | (pkt[11] & 0xFF);
		dev->y2 = ((pkt[14] & 0x0F) << 8) | (pkt[13] & 0xFF);
		dev->touch2 = pkt[8] & 0x03;
		dev->tag2 = pkt[10];
	}
	return 1;
}

static struct asmt_device_info type = {
		.min_xc		= 0x0,
		.max_xc		= 0x0fff,
		.min_yc		= 0x0,
		.max_yc		= 0x0fff,
		.rept_size	= 16,
		.read_data	= asus_read_data,
};

static void usbtouch_process_pkt(struct asus_mt_usb *asmt,
				 unsigned char *pkt, int len)
{
	struct asmt_device_info *type = asmt->type;

	if (!type->read_data(asmt, pkt))
			return;

	/*
	 * Begin to search for tag array which stores the event mapped to tag
	 * if touch is released then we must clear releated data.
	 */
	if (!asmt->touch1) {
		if (asmt->tag_array[asmt->tag1] == 1)
			asmt->input1_map_tag = 0;
		else
			asmt->input2_map_tag = 0;

		asmt->tag_array[asmt->tag1] = 0;
	}

	if (!asmt->touch2) {
		if (asmt->tag_array[asmt->tag2] == 1)
			asmt->input1_map_tag = 0;
		else
			asmt->input2_map_tag = 0;

		asmt->tag_array[asmt->tag2] = 0;
	}

	/*
	 * if tag1 already bind to event
	 */
	if (asmt->tag_array[asmt->tag1] != 0x0000) {
		if (asmt->tag_array[asmt->tag1] == 1) {
			input_report_key(asmt->input1, BTN_TOUCH, asmt->touch1);
			input_report_abs(asmt->input1, ABS_X, asmt->x1);
			input_report_abs(asmt->input1, ABS_Y, asmt->y1);
			input_sync(asmt->input1);
		} else {
			input_report_key(asmt->input2, BTN_TOUCH, asmt->touch1);
			input_report_abs(asmt->input2, ABS_X, asmt->x1);
			input_report_abs(asmt->input2, ABS_Y, asmt->y1);
			input_sync(asmt->input2);
		}
	} else {

		/*
		 * To find a event which is not used
		 */
		if (asmt->input1_map_tag != 0x0000) {
			asmt->tag_array[asmt->tag1] = 2;
			asmt->input2_map_tag = asmt->tag1;
			input_report_key(asmt->input2, BTN_TOUCH, asmt->touch1);
			input_report_abs(asmt->input2, ABS_X, asmt->x1);
			input_report_abs(asmt->input2, ABS_Y, asmt->y1);
			input_sync(asmt->input2);
		} else {
			asmt->tag_array[asmt->tag1] = 1;
			asmt->input1_map_tag = asmt->tag1;
			input_report_key(asmt->input1, BTN_TOUCH, asmt->touch1);
			input_report_abs(asmt->input1, ABS_X, asmt->x1);
			input_report_abs(asmt->input1, ABS_Y, asmt->y1);
			input_sync(asmt->input1);
		}
	}

	if (asmt->p2) {
		if (asmt->tag_array[asmt->tag2] != 0x0000) {
			if (asmt->tag_array[asmt->tag2] == 2) {
				input_report_key(asmt->input2, BTN_TOUCH,
						 asmt->touch2);
				input_report_abs(asmt->input2, ABS_X, asmt->x2);
				input_report_abs(asmt->input2, ABS_Y, asmt->y2);
				input_sync(asmt->input2);
			} else {
				input_report_key(asmt->input1, BTN_TOUCH,
						 asmt->touch2);
				input_report_abs(asmt->input1, ABS_X, asmt->x2);
				input_report_abs(asmt->input1, ABS_Y, asmt->y2);
				input_sync(asmt->input1);
			}
		} else {
			/*
			 * find a event which is not used
			 */
			if (asmt->input1_map_tag != 0x0000) {
				asmt->tag_array[asmt->tag2] = 2;
				asmt->input2_map_tag = asmt->tag2;
				input_report_key(asmt->input2, BTN_TOUCH,
						 asmt->touch2);
				input_report_abs(asmt->input2, ABS_X, asmt->x2);
				input_report_abs(asmt->input2, ABS_Y, asmt->y2);
				input_sync(asmt->input2);
			} else {
				asmt->tag_array[asmt->tag2] = 1;
				asmt->input1_map_tag = asmt->tag2;
				input_report_key(asmt->input1, BTN_TOUCH,
						 asmt->touch2);
				input_report_abs(asmt->input1, ABS_X, asmt->x2);
				input_report_abs(asmt->input1, ABS_Y, asmt->y2);
				input_sync(asmt->input1);
			}
		}
	}
}

#define USB_REQ_SET_REPORT 0x09
static int usb_set_report_feature(struct usb_interface *intf,
				  unsigned char type, unsigned char id,
				  void *buf, int size)
{
	return usb_control_msg(interface_to_usbdev(intf),
			       usb_sndctrlpipe(interface_to_usbdev(intf), 0),
			       USB_REQ_SET_REPORT,
			       USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			       (type << 8) + id,
			       intf->cur_altsetting->desc.bInterfaceNumber, buf,
			       size, HZ);
}

static void asus_mt_irq(struct urb *urb)
{
	int retval;
	struct asus_mt_usb *asmt = urb->context;
	struct device *dev = &asmt->interface->dev;

	switch (urb->status) {
	case 0:
		break;
	case -ETIME:
		dev_dbg(dev, "%s - urb timed out - was the device unplugged?",
		    __func__);
		return;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		goto resubmit;
	}

	asmt->type->process_pkt(asmt, asmt->data, urb->actual_length);
resubmit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_dbg(dev, "%s - usb_submit_urb failed with result: %d",
		    __func__, retval);
}

static int asus_probe(struct usb_interface *intf,
		      const struct usb_device_id *id)
{
	int ret = 0;
	struct device *temp_class;
	struct usb_host_interface *interface = intf->cur_altsetting;
	struct usb_device *dev = interface_to_usbdev(intf);
	struct input_dev *input_dev1, *input_dev2;
	struct asus_mt_usb *asmt;
	int n = 0;
	char *buf;
	int err = -ENOMEM;

	asus_mt_major = register_chrdev(0, ASUS_MT_STRING,
			&asus_mt_fops);

	if (asus_mt_major < 0) {
		dev_dbg(&intf->dev,
		"%s: Unable to get a major for ASUS multi-touch driver!\n",
		__func__);
		return asus_mt_major;
	}

	asus_mt_class = class_create(THIS_MODULE, ASUS_MT_STRING);

	if (IS_ERR(asus_mt_class)) {
		dev_dbg(&intf->dev,
			"%s: Error creating Asus Multi-Touch class.\n",
			__func__);
		ret = PTR_ERR(asus_mt_class);
		goto err_class_create;
	}

	temp_class = device_create(asus_mt_class, NULL,
					MKDEV(asus_mt_major, 0),
					NULL,  ASUS_MT_STRING);

	if (IS_ERR(temp_class)) {
		dev_dbg(&intf->dev,
			"%s: Error creating Asus Multi-Touch class device.\n",
			__func__);
		ret = PTR_ERR(temp_class);
		goto out_device_create;
	}

	asmt = kzalloc(sizeof(struct asus_mt_usb), GFP_KERNEL);
	input_dev1 = input_allocate_device();
	input_dev2 = input_allocate_device();

	if (!asmt || !input_dev1 || !input_dev2) {
		dev_dbg(&intf->dev, "%s: Memory is not enough\n", __func__);
		goto out_free;
	}

	asmt->type = &type;
	asmt->udev = dev;

	if (dev->manufacturer)
		strlcpy(asmt->name, dev->manufacturer, sizeof(asmt->name));

	if (dev->product) {
		if (dev->manufacturer)
			strlcat(asmt->name, " ", sizeof(asmt->name));
		strlcat(asmt->name, dev->product, sizeof(asmt->name));
	}

	if (!strlen(asmt->name))
		snprintf(asmt->name, sizeof(asmt->name),
			"USB Touchscreen %04x:%04x",
			 le16_to_cpu(dev->descriptor.idVendor),
			 le16_to_cpu(dev->descriptor.idProduct));

	usb_make_path(dev, asmt->phys0, sizeof(asmt->phys0));
	strlcat(asmt->phys0, "/input0", sizeof(asmt->phys0));
	usb_make_path(dev, asmt->phys1, sizeof(asmt->phys1));
	strlcat(asmt->phys1, "/input1", sizeof(asmt->phys1));

	if (!asmt->type->process_pkt) {
		dev_dbg(&intf->dev, "%s: process_pkt is null\n", __func__);
		asmt->type->process_pkt = usbtouch_process_pkt;
	}

	usb_set_intfdata(intf, asmt);
	input_dev1->name = asmt->name;
	input_dev2->name = asmt->name;
	usb_to_input_id(dev, &input_dev1->id);
	usb_to_input_id(dev, &input_dev2->id);
	asmt->input1 = input_dev1;
	asmt->input2 = input_dev2;


	input_dev1->dev.parent = &intf->dev;
	input_set_drvdata(input_dev1, asmt);

	input_dev1->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev1->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev1, ABS_X, asmt->type->min_xc,
			     asmt->type->max_xc, 0, 0);
	input_set_abs_params(input_dev1, ABS_Y, asmt->type->min_yc,
			     asmt->type->max_yc, 0, 0);

	input_dev2->dev.parent = &intf->dev;
	input_set_drvdata(input_dev2, asmt);

	input_dev2->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev2->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev2, ABS_X, asmt->type->min_xc,
			     asmt->type->max_xc, 0, 0);
	input_set_abs_params(input_dev2, ABS_Y, asmt->type->min_yc,
			     asmt->type->max_yc, 0, 0);

	asmt->data_size = type.rept_size;
	asmt->data = usb_alloc_coherent(dev, asmt->data_size, GFP_KERNEL,
			&asmt->data_dma);

	if (!asmt->data) {
		dev_dbg(&intf->dev, "%s: asmt->data allocating fail", __func__);
		goto out_free;
	}

	for (n = 0; n < interface->desc.bNumEndpoints; n++) {
		struct usb_endpoint_descriptor *endpoint;
		int pipe;
		int interval;

		endpoint = &interface->endpoint[n].desc;
		if (!usb_endpoint_xfer_int(endpoint))
			continue;

		interval = endpoint->bInterval;

		if (usb_endpoint_dir_in(endpoint)) {
			if (asmt->urb)
				continue;
			asmt->urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!asmt->urb)
				goto out_free_coherent;
			pipe = usb_rcvintpipe(dev, endpoint->bEndpointAddress);
			usb_fill_int_urb(asmt->urb, dev, pipe, asmt->data,
					 asmt->data_size, asus_mt_irq,
					 asmt, interval);
			asmt->urb->transfer_dma = asmt->data_dma;
			asmt->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		}
	}

	err = input_register_device(asmt->input1);
	if (err) {
		dev_dbg(&intf->dev,
			"%s - input_register_device failed, err: %d\n",
			__func__, err);
		goto out_input;
	}
	input_register_device(asmt->input2);
	if (err) {
		dev_dbg(&intf->dev,
			"%s - input_register_device failed, err: %d\n",
			__func__, err);
		goto out_input;
	}

	/* 8 bytes are enough for both products */
	buf = kmalloc(8, GFP_KERNEL);
	if (!buf)
		goto out_buf;

	buf[0] = 0x07;
	buf[1] = 0x02;
	if (usb_set_report_feature(intf, 2, 7, buf, 8))
		dev_dbg(&intf->dev, "%s: set report true\n", __func__);
	else
		dev_dbg(&intf->dev, "%s set report false\n", __func__);

	err = usb_submit_urb(asmt->urb, GFP_ATOMIC);
	if (err) {
		dev_err(&intf->dev,
			"%s: usb_submit_urb failed with error: %d\n",
			__func__, err);
		goto out_fail;
	}

out_buf:
	kfree(buf);
	return 0;

out_fail:
	input_unregister_device(input_dev2);
	input_unregister_device(input_dev1);
	input_dev2 = NULL;
	input_dev1 = NULL;
out_input:
	usb_free_urb(asmt->urb);
out_free_coherent:
	usb_free_coherent(dev, asmt->data_size,
			  asmt->data, asmt->data_dma);
out_free:
	input_free_device(input_dev1);
	input_free_device(input_dev2);
	kfree(asmt);
out_device_create:
	class_destroy(asus_mt_class);
err_class_create:
	unregister_chrdev(asus_mt_major, ASUS_MT_STRING);
	return err;
}

static void asus_disconnect(struct usb_interface *intf)
{
	struct asus_mt_usb *asmt = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	if (asmt) {
		input_unregister_device(asmt->input2);
		input_unregister_device(asmt->input1);
		usb_kill_urb(asmt->urb);
		usb_free_urb(asmt->urb);
		usb_free_coherent(interface_to_usbdev(intf), asmt->data_size,
				asmt->data, asmt->data_dma);
		kfree(asmt);
	}
}

static const struct usb_device_id asus_devices[] = {
	{ USB_DEVICE(USB_VENDOR_ID_ASUS, USB_DEVICE_ID_ASUS) },
	{ }
};
MODULE_DEVICE_TABLE(usb, asus_devices);

static struct usb_driver asus_driver = {
	.name = "asus",
	.probe = asus_probe,
	.disconnect = asus_disconnect,
	.id_table = asus_devices,
};

static int asus_init(void)
{
	return usb_register(&asus_driver);
}

static void asus_exit(void)
{
	usb_deregister(&asus_driver);
}

module_init(asus_init);
module_exit(asus_exit);

