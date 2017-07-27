
Primary Make All Build Targets:

  all:                     #  Configure and Build Everything for
			   #  the Initramfs Install.

  buildroot_production:	   #  Build Buildroot for Production; cleans
                           #  the old tree out.

  buildroot_production_clean:	   #  Build Buildroot for Production
                                   #  without cleaning old tree.

  synrad_legacy_all:       #  Build a tar file to install on legacy
                           #  system for testing upgrade.
 
  clean:                   #  Clean Everything

Buildroot Management Targets:

  buildroot:               #  Build Buildroot.  Assumes that the
                           #  configurtation has been installed
                           #  with the target 'buildroot_defconfig',
                           #  or that it has been modifie with the
                           #  target 'buildroot_xconfig'.  Used for
 			   #  NFS Root, or Production Root FS.

  buildroot_initramfs:     #  Build Buildroot.  Assumes that the
                           #  configurtation has been installed
                           #  with the target 'buildroot_initramfs_defconfig',
                           #  or that it has been modifie with the
                           #  target 'buildroot_xconfig'.  Used for
			   #  Linux Kernel for startup.

  buildroot_defconfig:     #  Install the default configuration from
                           #  ~/config/sros_buildroot_config.

  buildroot_initramfs_defconfig:   #  Install the default configuration from
                                   #  ~/config/sros_buildroot_intramfs_config.


  buildroot_saveconfig:    #  Saves current Buildroot configuration
                           #  into ~/config/sros_config.  Typically
                           #  used after running 'buildroot_xconfig',
                           #  and then testing the new Root FS.

  buildroot_intramfs_saveconfig:    #  Saves current Buildroot configuration
                                    #  into ~/config/sros_config.  Typically
                                    #  used after running 'buildroot_xconfig',
                                    #  and then testing the new Root FS.


  buildroot_xconfig:       #  Runs 'xconfig' with Buildroot.

  buildroot_clean:         #  Cleans Buildroot, including removing the
                           #  configuration file.  Use with care.

Kernel Management Targets:

  kernel:                  #  Builds the Kernel and DTB.

  kernel_defconfig:        #  Install the default configuration from
                           #  ~/config/sros_kernel_config.  Testing
			   #  only.

  kernel_initramfs_defconfig:   #  Install the default configuration from
                                #  ~/config/sros_initramfs_config.

  kernel_saveconfig:       #  Saves current Kernel configuration
                           #  into ~/config/sros_kernel_config.  Typically
                           #  used after running 'kernel_xconfig',
                           #  and then testing the new Kernel.

  kernel_initramfs_saveconfig: #  Saves current Kernel configuration
                               #  into ~/config/sros_initramfs_config.
			       #  Typically used after running
			       #  'kernel_xconfig', and then testing
	 		       #  the new Kernel.

  kernel_xconfig:          #  Runs 'xconfig' with the Kernel.

  kernel_mrproper:         #  Cleans Kernel, including removing the
                           #  configuration file.  Use with care.

  uboot:                   #  Configure and Build U-Boot. Built as
			   #  part of default build, simply to build
			   #  Linux Environment Handling tools.

  uboot_env:               #  Configure and Build U-Boot Linux Environment
			   #  tools.

Notes:

  All output goes into ~/output.  Example:

    tbesemer tbesemer 5638419 Jul 21 15:54 cuImage.yosemite
    tbesemer tbesemer 6902989 Jul 23 13:00 cuImage.yosemite.initramfs
    tbesemer tbesemer    1149 Jul 14 11:35 fw_env.config
    tbesemer tbesemer   32084 Jul 22 12:21 fw_printenv
    tbesemer tbesemer 1251028 Jul 23 13:02 rootfs_initramfs_kernel.cpio.gz
    tbesemer tbesemer 2754560 Jul 20 12:45 rootfs.tar

