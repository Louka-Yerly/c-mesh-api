/* Wirepas Oy licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include "platform.h"

#define LOG_MODULE_NAME "SERIAL"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"

static int fd = -1;

/** \brief uart device to use */
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(dect2020_dualmcu_uart));

/** \brief Forward declaration of internal open */
static int int_open();

/** \brief Bitrate to use */
static unsigned long m_bitrate;


static int set_interface_attribs(int fd, unsigned long bitrate, int parity)
{
	ARG_UNUSED(fd);
	ARG_UNUSED(parity);

    const struct uart_config uart_cfg = {
        .baudrate = bitrate,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
    int err = uart_configure(uart_dev, &uart_cfg);
    if (err < 0) {
        LOGE("Error while configuring uart (%i)", err);
        return -1;
    }

    /*
        potential problems:
            - // disable break processing
                tty.c_iflag &= ~IGNBRK;
            - // disable CR -> NL translation
                tty.c_iflag &= ~ICRNL;
            - // no signaling chars, no echo, no canonical processing
                tty.c_lflag = 0;
            - // no remapping, no delays
                tty.c_oflag = 0;
            - // VMIN=0, VTIME=1 => blocking for max 100ms
                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = 1;
            - // shut off xon/xoff ctrl
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            - // ignore modem controls, enable reading
                tty.c_cflag |= (CLOCAL | CREAD);
                tty.c_cflag &= ~(PARENB | PARODD);
            - // shut off parity
                tty.c_cflag |= parity;
                tty.c_cflag &= ~CSTOPB;
                tty.c_cflag &= ~CRTSCTS;
    */

    return 0;
}

static int int_open()
{
    if (!device_is_ready(uart_dev)) {
        LOGE("Error chekcing if device is ready (not ready)");
        return -1;
    }

    // set fd=3 for internal STMA 0=stin 1=stdout 2=stderr
    fd = 3;

    // set the requested bitrate, 8n1, no parity
    if (set_interface_attribs(fd, m_bitrate, 0) < 0)
    {
        fd = -1;
        return -1;
    }

    LOGD("Serial opened\n");
    return 0;
}
/****************************************************************************/
/*                Public method implementation                              */
/****************************************************************************/
int Serial_open(const char * port_name, unsigned long bitrate)
{
    // configured by the file prj.conf
    ARG_UNUSED(port_name);

    m_bitrate = bitrate;

    return int_open();
}

int Serial_close()
{
    fd = -1;
    LOGD("Serial closed\n");
    return 0;
}

static ssize_t get_single_char(unsigned char * c, unsigned int timeout_ms)
{
	int64_t starting = k_uptime_get();
	do {
		int ret = uart_poll_in(uart_dev, c);
		if(ret == 0) {
			return 1;
		}
	} while(k_uptime_delta(&starting) < timeout_ms);

	LOGD("Timeout to wait for char on serial line\n");
    return 0;
}

int Serial_read(unsigned char * c, unsigned int timeout_ms)
{
    if (fd < 0)
    {
        LOGE("No serial link opened\n");
        return -1;
    }

    return get_single_char(c, timeout_ms);
    return 1;
}

int Serial_write(const unsigned char * buffer, unsigned int buffer_size)
{
    if (fd < 0)
    {
        LOGE("No serial link opened\n");
        // Try to reopen
        if (int_open() < 0)
        {
            // Wait a bit before next try
			k_sleep(K_SECONDS(1));
            return 0;
        }
        LOGI("Serial reopened\n");
    }

	unsigned int sended_size = 0;
	while(sended_size < buffer_size) {
		uart_poll_out(uart_dev, buffer[sended_size++]);
	}

    return buffer_size;
}
