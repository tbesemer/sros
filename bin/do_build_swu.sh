#!/bin/bash

if [ ! -f $UPGRADE_SRC ]
then
    echo "UPGRADE_SRC $UPGRADE_SRC Does not exit"
    exit 0;
fi

pushd ${UPGRADE_STAGING}

cp -p $UPGRADE_SRC .

CONTAINER_VER="1.0.1"
PRODUCT_NAME="amcc"
FILES="sw-description rootfs.jffs2"

for i in $FILES;
do
    echo $i
done | cpio -ov -H crc >  ${PRODUCT_NAME}_${CONTAINER_VER}.swu

popd
exit 0
