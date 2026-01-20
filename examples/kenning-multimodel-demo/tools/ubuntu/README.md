# Ubuntu tools for Kenning Multimodel Demo

Directory contains tools prepared for running Kenning Multimodel Demo for Ubuntu and other Linux platforms running **systemd**, it consists of:
- install.sh - it is the script that will install service that allows for executing demo upon system boot
- kenning.multimodel.demo.service.template - it is a template file for **systemd** service used to start demo upon system boot, used by **install.sh** script

To install it for **Nvidia Jetson** platform type:
``` bash
./install.sh jetson
```

For any other platforms type:
``` bash
./install.sh
```