:: Deletes All files in the Current Directory With Prompts and Warnings 
::(Hidden, System, and Read-Only Files are Not Affected) 
:: 
@ECHO OFF 
python patch_image_header.py Build/Pegasus3.1v-app1.bin
python patch_image_header.py Build/Pegasus3.1v-app2.bin

openocd.exe -f ./openocd.cfg -c "program build/Pegasus3.1v-bootloader.bin reset verify exit 0x8000000"
openocd.exe -f ./openocd.cfg -c "program build/Pegasus3.1v-app1.bin reset verify exit 0x8040000"
openocd.exe -f ./openocd.cfg -c "program build/Pegasus3.1v-app2.bin reset verify exit 0x8080000"
