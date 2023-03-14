make all && arm-none-eabi-objcopy -O binary charIOT-Key-C.elf charIOT-Key-C.bin && sudo st-flash write charIOT-Key-C.bin 0x8000000 && sudo st-flash reset
