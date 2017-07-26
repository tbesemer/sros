#!/bin/sh

if [ ! -d $STAGING_DIR ]
then
    echo "STAGING_DIR <$STAGING_DIR> does not exist\n" 
    exit 1
fi

if [ ! -f $ROOTFS_INPUT ]
then
    echo "ROOTFS_INPUT <$ROOTFS_INPUT> does not exist\n" 
    exit 1
fi

#  Extract base from BuildRoot.
#
mkdir -p $STAGING_DIR/rootfs_tmp
pushd $STAGING_DIR/rootfs_tmp
if [ $? -ne 0 ]
then
   echo "Can't cd into temporary directory"
   exit 1
fi
rm -rf *
echo "Extracting Source ROOTFS"
tar xf $ROOTFS_INPUT

# Populate from Root FS
#
echo "Populating Root FS"
cp -p $OUTPUT_DIR/fw_printenv usr/bin/fw_printenv
cp -Rp $INITRAMFS_OVERLAY/etc/fw_env.config etc/
pushd usr/bin
ln -s fw_printenv fw_setenv
popd
mknod dev/mtd4 c 90 8
mknod dev/mtd5 c 90 10
mknod dev/mtdblock4 b 31 4
mknod dev/mtdblock5 b 31 5


#  Rebuild ROOTFS 
#
echo "Packaging Production ROOTFS"
tar cf "$ROOTFS_OUTPUT" *

popd

