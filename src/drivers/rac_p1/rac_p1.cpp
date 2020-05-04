/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RAC.cpp
 * Driver for the RAC on a serial/spi port
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#ifndef __PX4_QURT
#include <poll.h>
#endif

#include <termios.h>

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_cli.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <uORB/PublicationQueued.hpp>
#include <uORB/Subscription.hpp>
//#include <uORB/topics/GPS_dump.h>
//#include <uORB/topics/GPS_inject_data.h>



#ifdef __PX4_LINUX
#include <linux/spi/spidev.h>
#endif /* __PX4_LINUX */

#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000



class RAC : public ModuleBase<RAC>
{
public:
    /** The GPS allows to run multiple instances */
    enum class Instance : uint8_t {
        Main = 0,
        Secondary,

        Count
    };



    RAC(const char *path);
    virtual ~RAC();

    static int task_spawn(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[], Instance instance);

    static RAC *instantiate(int argc, char *argv[]);
    static RAC *instantiate(int argc, char *argv[], Instance instance);

    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    static int run_trampoline(int argc, char *argv[]);

    void run() override;
     int print_status() override;

    /**
     * Schedule reset of the GPS device
     */
    //void schedule_reset(GPSRestartType restart_type);

    /**
     * Reset device if reset was scheduled
     */
    //void reset_if_scheduled();


private:

    static int uart_init(char * uart_name);
    static int set_uart_baudrate(const int fd, unsigned int baud);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int rac_p1_main(int argc, char *argv[]);
int rac_p1_main(int argc, char *argv[])
{
    return RAC::main(argc, argv);
}

RAC::RAC(const char *path)
{
      printf(path);
}

RAC::~RAC()
{

}



//satrt RAC


int RAC::set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}

int RAC::uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

int RAC::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int RAC::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
GPS driver module that handles the communication with the device and publishes the position via uORB.
It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published
on the second uORB topic instance, but it's currently not used by the rest of the system (however the
data will be logged, so that it can be used for comparisons).

### Implementation
There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks
so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples
For testing it can be useful to fake a GPS signal (it will signal the system that it has a valid position):
$ gps stop
$ gps start -f

Starting 2 GPS devices (the main GPS on /dev/ttyS3 and the secondary on /dev/ttyS4):
$ gps start -d /dev/ttyS3 -e /dev/ttyS4

Initiate warm restart of GPS device
$ gps reset warm
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("rac_p1", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GPS device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);
    PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, "<file:dev>", "Optional secondary GPS device", true);
    PRINT_MODULE_USAGE_PARAM_INT('g', 0, 0, 3000000, "Baudrate (secondary GPS, can also be p:<param_name>)", true);

    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Fake a GPS signal (useful for testing)", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('s', "Enable publication of satellite info", true);

    PRINT_MODULE_USAGE_PARAM_STRING('i', "uart", "spi|uart", "GPS interface", true);
    PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "ubx|mtk|ash|eml", "GPS Protocol (default=auto select)", true);

    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset GPS device");
    PRINT_MODULE_USAGE_ARG("cold|warm|hot", "Specify reset type", false);

    return 0;
}

int RAC::print_status()
{
    PX4_INFO("RAC Start");
    return 0;
}


int RAC::task_spawn(int argc, char *argv[])
{
    return task_spawn(argc, argv, Instance::Main);
}

int RAC::task_spawn(int argc, char *argv[], Instance instance)
{
    px4_main_t entry_point;
    if (instance == Instance::Main) {
        entry_point = (px4_main_t)&run_trampoline;
    } else {
        entry_point = (px4_main_t)&run_trampoline;
    }

    int task_id = px4_task_spawn_cmd("rac", SCHED_DEFAULT,
                   SCHED_PRIORITY_SLOW_DRIVER, 1700,
                   entry_point, (char *const *)argv);

    if (task_id < 0) {
        task_id = -1;
        return -errno;
    }


    if (instance == Instance::Main) {
        _task_id = task_id;
    }
    return 0;
}

int RAC::run_trampoline(int argc, char *argv[])
{

#ifdef __PX4_NUTTX
    // on NuttX task_create() adds the task name as first argument
    argc -= 1;
    argv += 1;
#endif

    RAC *rac = instantiate(argc, argv, Instance::Main);
    if (rac) {
        rac->run();
        delete rac;
    }
    return 0;
}

void RAC::run()
{

}


RAC *RAC::instantiate(int argc, char *argv[])
{
    return instantiate(argc, argv, Instance::Main);
}

RAC *RAC::instantiate(int argc, char *argv[], Instance instance)
{
    const char *device_name = "/dev/ttyS3";
    int baudrate_main = 0;


    bool error_flag = false;
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "b:d:e:fg:si:p:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'b':
            if (px4_get_parameter_value(myoptarg, baudrate_main) != 0) {
                PX4_ERR("baudrate parsing failed");
                error_flag = true;
            }
            break;

        case 'd':
            device_name = myoptarg;
            break;

        case 'i':
            if (!strcmp(myoptarg, "spi")) {
                //interface = GPSHelper::Interface::SPI;

            } else if (!strcmp(myoptarg, "uart")) {
                //interface = GPSHelper::Interface::UART;

            } else {
                PX4_ERR("unknown interface: %s", myoptarg);
                error_flag = true;
            }
            break;

        case 'p':
            if (!strcmp(myoptarg, "ubx")) {
                //mode = GPS_DRIVER_MODE_UBX;

            } else if (!strcmp(myoptarg, "mtk")) {
                //mode = GPS_DRIVER_MODE_MTK;

            } else if (!strcmp(myoptarg, "ash")) {
                //mode = GPS_DRIVER_MODE_ASHTECH;

            } else if (!strcmp(myoptarg, "eml")) {
                //mode = GPS_DRIVER_MODE_EMLIDREACH;

            } else {
                PX4_ERR("unknown interface: %s", myoptarg);
                error_flag = true;
            }
            break;

        case '?':
            error_flag = true;
            break;

        default:
            PX4_WARN("unrecognized flag");
            error_flag = true;
            break;
        }
    }

    if (error_flag) {
        return nullptr;
    }
    RAC *rac = nullptr;
    if (instance == Instance::Main) {
        rac = new RAC(device_name);
    }

    return rac;
}





