# I2C Manager for ESP32

**Fully thread-safe I2C bus communication ESP-IDF component.**

&nbsp;


## The Problem

### It's a mess

The ESP32's I2C drivers work, but they are built for one piece of code to open, use and close a port. Currently configuring components that talk to peripherals on an I2C bus involves providing the same SDA and SDC pins and clock speeds again and again. Everyone ends up duplicating the same string of low-level commands to talk to the ESP32 I2C driver. What's needed is a hardware abstraction layer (HAL) that can just read and write bytes to I2C connected hardware, plain and simple.

But we need more than that: when components actually want to use the same I2C bus, they will get errors when the port is already open. People sometimes end up editing component code to ignore the error from `i2c_driver_install` in all but the first component initialised.

### "The Multi-Thread Issue"

An I2C bus in a modern piece of hardware is shared by any number of chips inside. Once the components ignore the error in opening the port, things often work. They might be talking a few bytes to a power management IC, set a backlight brightness, all from the same task, and that was it. But in other cases, there might be a touch panel and a 3D accelerometer on the same bus. These are both components you may want to read many times a second. However, according to Espressif's official ESP-IDF documentation:

&nbsp;

[<img alt="The I2C APIs are not thread-safe, if you want to use one I2C port in different tasks, you need to take care of the multi-thread issue." src="readme_assets/not-safe.png" width=80%>](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#_CPPv420i2c_master_cmd_begin10i2c_port_t16i2c_cmd_handle_t10TickType_t)

&nbsp;

So what we need is not just hardware abstraction, but also a way to prevent different ESP-IDF components (or tasks within one component) from reading or writing corrupted I2C data by trying to use the I2C port at the same time.

### Existing Solutions

I'm not aware of thread-safe abstractions, but there are hardware abstraction layers for I2C. I2C Manager started as a fork of Mika Tuupola's [`esp_i2c_helper`](https://github.com/tuupola/esp_i2c_helper). I2C Manager is compatible with it, so any drivers written by Mika or others using his hardware abstraction standard will just work. There are a few different ways to use I2C manager, but one of them is to use it as a drop-in replacement for I2C Helper that makes everything thread-safe.

&nbsp;


## I2C Manager

### Simplest Usage

Clone this repository in your components directory, use menuconfig to enable and configure one or both the I2C ports:

<img alt="menuconfig screenshot" src="readme_assets/menuconfig.png" width=70%>

Then in your own code do something like:

```c
#include "i2c_manager.h"

uint8_t buffer[10];
i2c_manager_read(I2C_NUM_0, 0x23, 0x42, &buffer, 10);
i2c_manager_write(I2C_NUM_0, 0x13, 0x37, &buffer, 10);
```

This example reads ten bytes via the first I2C port from the chip at I2C address `0x23`, register `0x42` and then writes them to the chip at `0x13` at register `0x37`. You could now have ten different tasks reading and writing to the same I2C bus using I2C Manager and the FreeRTOS scheduler would make them all wait nicely for their turn at the port.

That is it, all there is to it. As you can see there is no initialisation needed: the first time someone acesses an I2C port it will be set up automatically.

### Timeouts and pullups

As you can see in the screenshot above, the port setup for I2C manager allows for the usual GPIO pins for the SDA and SCL signals as well as the clock frequency in Hz. The additional settings allow you to to set how soon peripherals on the bus must respond before the read or write calls time out as well as how long another call must maximally wait before overriding the lock on a port. Finally you can specify whether or not the ESP32's internal pull-up resistors should be used. You can safely leave all these values at their defaults.

&nbsp;


### Using I2C Manager In/With Other ESP-IDF Components

#### One Way: Depending On I2C Manager

If your new driver is an ESP-IDF component that always needs to talk via I2C, the cleanest and easiest thing is to make your component depend on `i2c_manager`. This is done by adding `PRIV_REQUIRES i2c_manager` to your `idf_component_register` statement in `CMakeLists.txt` (for ESP-IDF 4.x) and/or adding `COMPONENT_DEPENDS := i2c_manager` to `component.mk` (for ESP-IDF 3.x). Then tell users to clone this repository to their `components` directory and in your code do `#include "i2c_manager.h"` in the code (and not the header file) and talk to the I2C port as in the example at the top of this README.

* Advantage: People that include your component's .h file(s) don't need to think about anything: you could maybe ask what port they want to use in your `Kconfig` file, and otherwise everything just works.

* Disadvantage: Your component must now depend on I2C Manager. That means it needs to be installed even if your component only talks to I2C in some configurations and not in others. Imagine a display driver that can work with tons of displays and touch interfaces, only some of which use I2C. But ESP-IDF does not allow for conditional dependencies, so everyone has to install I2C Manager.

> Note: if your code has applications outside of the ESP32 platform, you may want to consider writing it "platform agnostic" and providing an I2C HAL interface. See below for more details.

&nbsp;


#### Other Way: Integrating I2C Manager Inside Your Component

[LVGL - Light and Versatile Graphics Library](https://github.com/lvgl/lvgl) is a graphical interface library that I can recommend. I2C Manager has been integrated in the [ESP32 driver package](https://github.com/lvgl/lvgl_esp32_drivers) `lvgl_esp_drivers`. This is = an example of the procedure outlined below, because this is the case as described above: a display and touch interface that only in some configurations need to talk to hardware via I2C. To integrate I2C Manager inside your component:

1. Copy the inner `i2c_manager` directory (i.e. `i2c_manager/i2c_manager` to the root of your component.

2. Uncomment the `#define` in the i2c_manager.h file within that directory to name your component:

	`#define I2C_OEM xyz`   <--- use some short name of your component here

3. Make sure the `i2c_manager.c` file compiles when your component gets built, even though it's in a sub-directory. This is done by adding `"i2c_manager/i2c_manager.c"` to the SRCS directive of the `idf_component_register` command (in ESP-IDF 4.x) and/or adding `./i2c_manager` to `COMPONENT_SRCDIRS` in `component.mk` (for ESP-IDF 3.x).

	> Note: It's important that this `i2c_manager` directory is **not** added to the include directories for your component.

4. Add the following to your component's `componentXYZ.h` file:

	`void xyz_i2c_locking(void* leader);`

5. Add the following inside your component's Kconfig menu:

	`rsource "i2c_manager/Kconfig"`

And voila: now I2C Manager is integrated inside the component, which now has built-in I2C support without external dependencies. To use it, use `xyz_i2c_read` where you would otherwise use `i2c_manager_read` and the same for `xyz_i2c_write`. The menuconfig will show the settings for both I2C ports. You may want to include a configuration option to determine which I2C port your component uses. If multiple components integrate I2C Manager in this way, the I2C port settings menu will show under each component, but they will show the same settings. A port open error will occur if multiple components use the same port, but I2C manager will just log a warning and continue when getting this error.

**Making things thread-safe:** To make sure the component is thread-safe when others call I2C ports, users can opt to put I2C Manager in the components directory, and tell your component to use I2C Manager's locking. In the main program, after including both `i2c_manager.h` and your component's `componentXYZ.h` file, this is done like this:

  `componentXYZ_locking(i2c_manager_locking());`

When I2C Manager is installed, the individual I2C port settings menus with components that have integrated I2C Manager will disappear, there will only be the one port settings menu for I2C Manager.

* Advantages: If you integrate I2C Manager like this, there are no external dependencies and everything just works.

* Disadvantages: While any tasks within your component are thread-sage relative to one another, users still have to install I2C Manager and tell your component to play nice with it (like described above) to get thread-safety between components.

&nbsp;


#### Third Way: I2C HAL

<img src="readme_assets/tuupola-hal.png" width=80%>

Sometimes software for a particular chip has no good reason to be written only for the ESP32. Mika Tuupola [(@tuupola)](https://github.com/tuupola) has written quite a few "platform agnostic I2C drivers" for various ICs. To abstract I2C communication, he defines a way of providing a pointer to a struct that holds pointers to functions to read and write to I2C. To make his software use I2C Manager could not be simpler. Wherever you would provide a pointer in his examples, simply provide `i2c_hal(0)` for port 0 or `i2c_hal(1)` for port 1. That's it. So for instance to talk to the AXP192 Power Management IC using his [driver](https://github.com/tuupola/axp192), you would do:

```c
#define I2C_0	i2c_hal(0)
#include "axp192.h"

float cbat;
axp192_read(I2C_0, AXP192_COULOMB_COUNTER, &cbat);
printf("cbat: %.2fmAh", cbat);
```

When you do this, everything is thread-safe. I2C Manager replaces Mika's [`esp_i2c_helper`](https://github.com/tuupola/esp_i2c_helper) component, and as a bonus is slightly easier to use in your own code because you do not have to futz with the HAL struct yourself.

>Note that the I2C HAL as defined does not support 10-bit addresses or 16-bit registers (see below).

&nbsp;


### 10-bit addresses and 16-bit registers

In most cases, I2C uses 7-bit addresses and an 8-bit register value. You do not need to do anything special in that case. On some ICs you read or write without specifying a register. In that case, use 0 as the register value.

Sometimes you'll see an 8-bit values for the address, usually with a separate address for read and write. Although this is how the byte is actually sent in the I2C protocol (the address is shifted left and a read/write bit is added), documenting it this way is deeply wrong and the address should be interpreted as 7-bit address by dividing the lower number by two.

I2C addresses can also be 10-bit values with some ICs. To signal the use of 10-bit addresses, bitwise-OR the address value you send to `i2c_manager_read` or `i2c_manager_write` with `I2C_ADDR_10`. Similarly, to use 16-bit register values, bitwise-OR the register value with `I2C_REG_16`.

&nbsp;

### I2C Manager Function Reference

#### read, write

```c
esp_err_t i2c_manager_read(i2c_port_t port, uint16_t addr, uint32_t reg, uint8_t *buffer, uint16_t size);

esp_err_t i2c_manager_write(i2c_port_t port, uint16_t addr, uint32_t reg, const uint8_t *buffer, uint16_t size);
```

Straightforward: read or write `size` bytes between `buffer` and register `reg` of the chip at I2C address `address`. Most likely these are the only functions you will ever need to call, everything else happens automatically. See the example all the way at the beginning of this text for an example of how to use them.

> Note that these exact same functions might be called `xyz_i2c_read` and `xyz_i2c_write` when I2C Manager is integrated in componentXYZ.


#### locking

```c
void* i2c_manager_locking();
```

You will only need this if you use a component that has I2C Manager integrated (See "Other Way" above). This returns a pointer to the two semaphore handles that make up the mutex locking mechanism for I2C manager. Never mind if that doesn't mean anything to you: simply provide this pointer to the `xyz_locking` function of the component and their I2C communication will play nice with everyone else's.

#### init     &nbsp;&nbsp; *(you don't need this)*

```c
esp_err_t i2c_manager_init(i2c_port_t port);
```

You can call `i2c_manager_init` for a port if you like and you can call it as often as you like. It will just do nothing and return if the port is already open. In fact, this is what happens at every `read` or `write` call, so there's really no reason for you to be calling this yourself.


#### close     &nbsp;&nbsp; *(nor will you need this)*

```c
esp_err_t i2c_manager_close(i2c_port_t port);
```

You can call this if you insist, but the next one to call read or write on the port is just going to cause it to open back up.

#### manual locking     &nbsp;&nbsp; *(and you don't need this either)*

```c
esp_err_t i2c_manager_lock(i2c_port_t port);

esp_err_t i2c_manager_unlock(i2c_port_t port);

esp_err_t i2c_manager_force_unlock(i2c_port_t port);
```

These are also functions you will likely never need to call. But in the unlikely event that you need to do something unusual with the I2C port that involves your own low-level code, you can call `i2c_manager_lock` before you do you own thing, and `i2c_manager_unlock` after you are done to make sure that none of the other code that is using I2C Manager can interfere.

`i2c_manager_force_unlock` can free up a lock that was set by code that didn't release it, maybe because it terminated unexpectedly. If you are going to write code that locks, make sure that nothing unexpected can happen that causes your code to not release the lock, and make sure only the actual handling of the port happens between lock and unlock.

&nbsp;
