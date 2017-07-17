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

#  Rebuild CPIO File for Kernel
#
echo "Packaging Initramfs CPIO"
find . | cpio -o > $CPIO_OUTPUT

popd

