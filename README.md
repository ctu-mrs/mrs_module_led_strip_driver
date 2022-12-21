## mrs_module_led_strip_driver

Driver for MRS LED module. This allows turning on and off the LED lights as well as the included 12V DC power outlet.
This driver uses the MRS LLCP protocol
* See [mrs_llcp_ros](https://github.com/ctu-mrs/mrs_llcp_ros)

# Requirements
* The module microcontroller should have the [firmware](https://github.com/ctu-mrs/mrs_module_led_strip_driver/blob/master/firmware/firmware.ino) installed.
* The LLCP mesages are defined in the attached [header file](https://github.com/ctu-mrs/mrs_module_led_strip_driver/blob/master/firmware/msgs.h).
* Linked and compiled LED driver [nodelet](https://github.com/ctu-mrs/mrs_module_led_strip_driver) in your [workspace](https://ctu-mrs.github.io/docs/system/preparing_for_a_real-world_experiment.html#set-up-your-own-workspace).
* Linked and compiled MRS LLCP [nodelet](https://github.com/ctu-mrs/mrs_llcp) in modules workspace.
* Correctly set  `.rules` file in `/etc/udev/rules.d`, the LED module portname is `led_strip_driver`
  * e.g. 
  ```bash
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="led_strip_driver",OWNER="YOUR_LOGIN",MODE="0666"
  ```
* Wire connection between the onboard computer and distribution board

# How to run

