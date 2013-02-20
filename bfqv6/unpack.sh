./split_bootimg.pl boot.img
./unpack-bootimg.pl -i boot.img
mkdir ramdisk
cd ramdisk
lzma -dc ../boot.img-ramdisk.gz | cpio -i
cd ../
