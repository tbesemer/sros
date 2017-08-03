#!/bin/sh

if [ ! -d $STAGING_DIR ]
then
    echo "STAGING_DIR <$STAGING_DIR> does not exist\n" 
    exit 1
fi

if [ ! -f $ROOTFS_OUTPUT ]
then
    echo "ROOTFS_OUTPUT <$ROOTFS_OUTPUT> does not exist\n" 
    exit 1
fi

#  Build up Core
#
mkdir -p $STAGING_DIR/legacy_files
pushd $STAGING_DIR/legacy_files
if [ $? -ne 0 ]
then
   echo "Can't cd into temporary directory"
   exit 1
fi
rm -rf *

# Populate from Legacy Updates
#
echo "Populating Legacy Updates"
mkdir etc
mkdir home
mkdir -p usr/bin
cp -p $ROOTFS_OUTPUT home/
cp -p $INITRAMFS_KERNEL home/
cp -p $LEGACY_STARTUP home/
cp -Rp $UBOOT_FW_ENV etc/
cp -p $OLD_UBOOT_TOOL usr/bin/fw_printenv
cp -p $SYNRAD_UBOOT home/u-boot.bin
pushd usr/bin
ln -s fw_printenv fw_setenv
popd
chown -R root:root *

#  Package up Legacy
#
echo "Packaging Legacy Tar File"
tar cf "$LEGACY_OUTPUT" *

popd

