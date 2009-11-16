/******************************************************************************
 * AsusTsUsb.c  --  Driver for Multi-Touch USB Touchscreens
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

#define USB_VENDOR_ID_ASUS 0x0486
#define USB_DEVICE_ID_ASUS 0x0185
#define VAIO_RDESC_CONSTANT 0x0001

MODULE_LICENSE("GPL");

static int swap_xy;
module_param(swap_xy, bool, 0644);
MODULE_PARM_DESC(swap_xy, "If set X and Y axes are swapped.");

struct asus_mt_usb {
	unsigned char *data;
	struct	input_dev *input1, *input2;
	struct asmt_device_info *type;
	struct usb_device *udev;
	dma_addr_t data_dma;
	struct urb *urb;
	char name[128];
	char phys0[64];
	char phys1[64];
	int x1, y1;
	int x2, y2;
	int touch, press;
	int p2;
};
struct asmt_device_info {
	int min_xc, max_xc;
	int min_yc, max_yc;
	int min_press, max_press;
	int rept_size;

	void (*process_pkt) (struct asus_mt_usb *asmt, unsigned char *pkt, int len);

	/*
	 * used to get the packet len. possible return values:
	 * > 0: packet len
	 * = 0: skip one byte
	 * < 0: -return value more bytes needed
	 */
	int  (*get_pkt_len) (unsigned char *pkt, int len);

	int  (*read_data)   (struct asus_mt_usb *asmt, unsigned char *pkt);
	int  (*init)        (struct asus_mt_usb *asmt);
};
static int asus_read_data(struct asus_mt_usb *dev, unsigned char *pkt)
{
	/*
	if (!(pkt[0] & 0x80) || ((pkt[1] | pkt[2] | pkt[3]) & 0x80))
		return 0;

	dev->x = ((pkt[0] & 0x1F) << 7) | (pkt[2] & 0x7F);
	dev->y = ((pkt[1] & 0x1F) << 7) | (pkt[3] & 0x7F);
	dev->touch = pkt[0] & 0x20;
	*/
	dev->x1 = ((pkt[4] & 0x0F) << 8) | (pkt[3] & 0xFF);
	dev->y1 = ((pkt[6] & 0x0F) << 8) | (pkt[5] & 0xFF);
	
	dev->p2 = pkt[13] & 0x02;

	if(dev->p2) {
		dev->x2 = ((pkt[10] & 0x0F) << 8) | (pkt[9] & 0xFF);
		dev->y2 = ((pkt[12] & 0x0F) << 8) | (pkt[11] & 0xFF);
	}
	return 1;
}
static struct asmt_device_info type = {
		.min_xc		= 0x0,
		.max_xc		= 0x0fff,
		.min_yc		= 0x0,
		.max_yc		= 0x0fff,
		.rept_size	= 8,
		.read_data	= asus_read_data,
};

static void usbtouch_process_pkt(struct asus_mt_usb *asmt,
                                 unsigned char *pkt, int len)
{
	struct asmt_device_info *type = asmt->type;

	if (!type->read_data(asmt, pkt))
			return;

	input_report_key(asmt->input1, BTN_TOUCH, asmt->touch);
	input_report_key(asmt->input2, BTN_TOUCH, asmt->touch);

	if (swap_xy) {
		input_report_abs(asmt->input1, ABS_X, asmt->y1);
		input_report_abs(asmt->input1, ABS_Y, asmt->x1);
		input_report_abs(asmt->input2, ABS_X, asmt->y2);
		input_report_abs(asmt->input2, ABS_Y, asmt->x2);
	} else {
		input_report_abs(asmt->input1, ABS_X, asmt->x1);
		input_report_abs(asmt->input1, ABS_Y, asmt->y1);

		if(asmt->p2) {
			input_report_abs(asmt->input2, ABS_X, asmt->x2);
			input_report_abs(asmt->input2, ABS_Y, asmt->y2);
		}
	}
	/*
	 * if (type->max_press)
	 *
	 * input_report_abs	input_report_abs(asmt->input, ABS_PRESSURE, asmt->press);
	 */
	input_sync(asmt->input1);
	if(asmt->p2)
		input_sync(asmt->input2);
}

#define USB_REQ_SET_REPORT 0x09
static int usb_set_report_feature(struct usb_interface *intf, unsigned char type,
			  unsigned char id, void *buf, int size)
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
	//printk("%s: that's ok\n", __FUNCTION__ );
	struct asus_mt_usb *asmt = urb->context;
	//struct usbhid_device *usbhid = hid->driver_data;
	int retval;
	unsigned char* data = asmt->data;
	switch (urb->status) {
	case 0:
		// success 
		break;
	case -ETIME:
		// this urb is timing out 
		dbg("%s - urb timed out - was the device unplugged?",
		    __FUNCTION__);
		return;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		goto resubmit;
	}

	printk("data = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", data[0],\
			data[1],data[2],data[3],\
			data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11],data[12],data[13]);
	//printk("%s:urb->actual_length = %d\n", __FUNCTION__, urb->actual_length );
	asmt->type->process_pkt(asmt, asmt->data, urb->actual_length);
	resubmit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		err("%s - usb_submit_urb failed with result: %d",
		    __FUNCTION__, retval);
}

static int asus_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	printk("%s\n", __FUNCTION__);
	struct usb_host_interface *interface = intf->cur_altsetting;
	struct usb_device *dev = interface_to_usbdev(intf);
	struct input_dev *input_dev1, *input_dev2;
	int n = 0, insize = 14;

	struct asus_mt_usb *asmt = kzalloc(sizeof(struct asus_mt_usb), GFP_KERNEL);
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
		printk("process_pkt is null\n");
		asmt->type->process_pkt = usbtouch_process_pkt;
	}
	usb_set_intfdata(intf, asmt);
	input_dev1 = input_allocate_device();
	input_dev2 = input_allocate_device();
	input_dev1->name = asmt->name;
	input_dev2->name = asmt->name;
	usb_to_input_id(dev, &input_dev1->id);
	usb_to_input_id(dev, &input_dev2->id);
	asmt->input1 = input_dev1;
	asmt->input2 = input_dev2;
	if(!asmt || !input_dev1 || !input_dev2) {
		printk("Memory is not enough\n");
		goto fail1;
	}

	input_dev1->dev.parent = &intf->dev;
	input_set_drvdata(input_dev1, asmt);

	input_dev1->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev1->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev1, ABS_X, asmt->type->min_xc, asmt->type->max_xc, 0, 0);
	input_set_abs_params(input_dev1, ABS_Y, asmt->type->min_yc, asmt->type->max_yc, 0, 0);

	input_dev2->dev.parent = &intf->dev;
	input_set_drvdata(input_dev2, asmt);

	input_dev2->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev2->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev2, ABS_X, asmt->type->min_xc, asmt->type->max_xc, 0, 0);
	input_set_abs_params(input_dev2, ABS_Y, asmt->type->min_yc, asmt->type->max_yc, 0, 0);

	asmt->data = usb_buffer_alloc(dev, insize, GFP_KERNEL,
			&asmt->data_dma);
	if(!asmt->data) {
		printk("asmt->data allocating fail");
		goto fail;
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
			if (!(asmt->urb = usb_alloc_urb(0, GFP_KERNEL)))
				goto fail;
			pipe = usb_rcvintpipe(dev, endpoint->bEndpointAddress);
			//insize = usb_maxpacket(dev, pipe, usb_pipeout(pipe));
			usb_fill_int_urb(asmt->urb, dev, pipe, asmt->data, 
					 insize, asus_mt_irq, asmt, interval);
			asmt->urb->transfer_dma = asmt->data_dma;
			asmt->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		} 
	}
	
	char *buf = kmalloc(8, GFP_KERNEL);	/* 8 bytes are enough for both products */
	buf[0] = 0x07;
	buf[1] = 0x02;
	if(usb_set_report_feature(intf, 2, 7, buf, 8))
		printk("set report true\n");
	else
		printk("set report false\n");

	if (usb_submit_urb(asmt->urb, GFP_ATOMIC)) {
		printk("usb submit urb error\n");
		return -EIO;
	}

	input_register_device(asmt->input1);
	input_register_device(asmt->input2);
	return 0;
fail:
	usb_free_urb(asmt->urb);
	asmt->urb = NULL;
	usb_buffer_free(dev, insize, asmt->data, asmt->data_dma);
fail1:
	input_free_device(input_dev1);
	input_free_device(input_dev2);
	kfree(asmt);
	return 1;
}

static void asus_disconnect(struct usb_interface *intf)
{
	struct asus_mt_usb *asmt = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	if (asmt) {
		input_unregister_device(asmt->input2);
		input_unregister_device(asmt->input1);
		usb_kill_urb(asmt->urb);
		//input_unregister_device(asmt->input);
		usb_free_urb(asmt->urb);
		usb_buffer_free(interface_to_usbdev(intf), 14,
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
	printk("asus_init\n");
	return usb_register(&asus_driver);
}

static void asus_exit(void)
{
	printk("asus_exit\n");
	usb_deregister(&asus_driver);
}

module_init(asus_init);
module_exit(asus_exit);

