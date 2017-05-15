
Primary Make All Build Targets:

  all:                     #  Configure and Build Everything
  clean:                   #  Clean Everything

Buildroot Management Targets:

  buildroot:               #  Build Buildroot.  Assumes that the
                           #  configurtation has been installed
                           #  with the target 'buildroot_defconfig',
                           #  or that it has been modifie with the
                           #  target 'buildroot_xconfig'.

  buildroot_defconfig:     #  Install the default configuration from
                           #  ~/config/sros_config.

  buildroot_saveconfig:    #  Saves current Buildroot configuration
                           #  into ~/config/sros_config.  Typically
                           #  used after running 'buildroot_xconfig',
                           #  and then testing the new Root FS.

  buildroot_xconfig:       #  Runs 'xconfig' with Buildroot.

  buildroot_clean:         #  Cleans Buildroot, including removing the
                           #  configuration file.  Use with care.

Kernel Management Targets:

  kernel:                  #  Builds the Kernel and DTB.

  kernel_defconfig:        #  Install the default configuration from
                           #  ~/config/sros_kernel_config.

  kernel_saveconfig:       #  Saves current Kernel configuration
                           #  into ~/config/sros_kernel_config.  Typically
                           #  used after running 'kernel_xconfig',
                           #  and then testing the new Kernel.

  kernel_xconfig:          #  Runs 'xconfig' with the Kernel.

  kernel_mrproper:         #  Cleans Kernel, including removing the
                           #  configuration file.  Use with care.

Notes:

  All output goes into ~/output.  Example:

    tbesemer@t4linux4:~/src/sros-0514a/sros$ ls -l output
    total 7428
    -rw-r--r-- 1 tbesemer tbesemer 1587200 May 14 17:17 rootfs.tar
    -rw-rw-r-- 1 tbesemer tbesemer 6008135 May 14 17:36 uImage
    -rw-rw-r-- 1 tbesemer tbesemer    7063 May 14 17:36 yosemite.dtb


