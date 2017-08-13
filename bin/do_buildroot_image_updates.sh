#!/bin/sh

echo "Besemer CHROOT Hook"
pushd ../
ROOT_DIR=`pwd`
popd
echo "Target Root FS: $1"
echo "SROS Root: $ROOT_DIR"

exit 0

