#!/bin/sh

if [ ! -d $STAGING_DIR ]
then
    echo "STAGING_DIR <$STAGING_DIR> does not exist\n" 
    exit 1
fi

if [ ! -f $CPIO_INPUT ]
then
    echo "CPIO_INPUT <$CPIO_INPUT> does not exist\n" 
    exit 1
fi

#  Extract base from BuildRoot.
#
mkdir -p $STAGING_DIR/initramfs_tmp
pushd $STAGING_DIR/initramfs_tmp
echo "Extracting Initramfs Source CPIO"
cpio -id  < $CPIO_INPUT

# Populate from Template
#
echo "Populating Initramfs"
cp -Rp $INITRAMFS_OVERLAY/* .
cp -p $OUTPUT_DIR/fw_printenv usr/bin/fw_printenv
pushd usr/bin
ln -s fw_printenv fw_setenv
popd
mknod dev/mtd4 c 90 8
mknod dev/mtd5 c 90 10
mknod dev/mtdblock4 b 31 4
mknod dev/mtdblock5 b 31 5

#  Rebuild CPIO File for Kernel
#
echo "Packaging Initramfs CPIO"
find .  | cpio -H newc -o > "$CPIO_OUTPUT".tmp
cat "$CPIO_OUTPUT".tmp | gzip > "$CPIO_OUTPUT".igz

popd

