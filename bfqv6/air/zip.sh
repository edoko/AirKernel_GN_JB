air_ver="Air_Kernel_Weekly_r510_4.2.x";
home="/home/edoko/edoko-jb/mkboot/";
cp boot.img /zipackage
cd zipackage
zip -r `echo $air_ver`.zip *
#rm -rf $home/`echo $air_ver`.zip
cp `echo $air_ver`.zip $home/air_kernel/
if [ ! -d ../zip ]; then
 mkdir ../zip
fi
mv `echo $air_ver`.zip ../zip/
#rm -rf `echo $air_ver`.zip boot.img
cd ..

echo Success zipackage
