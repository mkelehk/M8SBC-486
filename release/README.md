- 需要烧录的固件有2个：  
AVR固件  - fpga_loader.hex 或 fpga_loader_verilog.hex  
BIOS固件 - m8sbc_flash.bin  

- AVR烧录命令：  
avrdude.exe -c usbasp -p m128 -U flash:w:fpga_loader.hex:i -U lfuse:w:0x3F:m -U hfuse:w:0xD7:m -U efuse:w:0xFF:m  

- 板子配置  
J6排针：需要跳线帽短接  
J9排针：需要跳线帽短接，管脚拉低。拉高貌似也没影响  
J16排针: AVR调试串口，波特率为57600,8N1  
