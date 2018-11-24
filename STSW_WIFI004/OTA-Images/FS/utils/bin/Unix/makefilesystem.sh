#!/bin/bash
#

print_usage() {
	echo "usage:"
	echo "   $0 -h [--image-size={image_size[K|M]}] [--image-name={image_name}] [--fat-size={fat_size}] [--source-dir={source_dir}]"
}

IMAGE_SIZE=64K
IMAGE_NAME=SPWF04S-filesystem.img
SOURCE_DIR=../../../APP_Disk
FAT_SIZE=12

TEMP=`getopt -o h:: --long image-size:,image-name:,fat-size:,source-dir: \
	-n 'makefilesystem.sh' -- "$@"`

if [ $? != 0 ] ; then echo "Terminating..." >&2 ; exit 1 ; fi

# Note the quotes around `$TEMP': they are essential!
eval set -- "$TEMP"

while true ; do
	case "$1" in
		-h) print_usage ; exit 0 ;;
		--image-size) IMAGE_SIZE=$2 ; shift 2 ;;
		--image-name) IMAGE_NAME="$2" ; shift 2 ;;
		--fat-size)   FAT_SIZE=$2 ;   shift 2 ;;
		--source-dir) SOURCE_DIR=$2 ; shift 2 ;;
		--) shift ; break ;;
		*) echo "Internal error!" ; exit 1 ;;
	esac
done

echo image size = $IMAGE_SIZE
echo image name = $IMAGE_NAME
echo source dir = $SOURCE_DIR
echo fat size   = $FAT_SIZE

echo creating image disk...
dd if=/dev/zero of=$IMAGE_NAME bs=$IMAGE_SIZE count=1

echo making the FAT$FAT_SIZE filesystem: $IMAGE_NAME
mkfs.msdos -F $FAT_SIZE $IMAGE_NAME

echo mounting the filesystem...
export FS_MOUTPOINT=`mktemp -d mountpoint.XXXXXX`
sudo mount $IMAGE_NAME $FS_MOUTPOINT

echo copying files to image disk...
sudo cp $SOURCE_DIR/* $FS_MOUTPOINT
sync

echo unmountng the image disk...
sudo umount $FS_MOUTPOINT

echo removing the mountpoint...
rmdir $FS_MOUTPOINT

echo done!
echo Now copy the file to webserver with the command:
echo scp $IMAGE_NAME webserver@192.168.1.129:/var/www
