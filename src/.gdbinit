# Connect to OpenOCD
target remote | openocd -f /usr/share/openocd/scripts/board/frdm-k64f.cfg -c "gdb_port pipe; log_output openocd.log"
# Reset the target and call its init script
monitor reset init
# Halt the target. The init script should halt the target, but just in case
monitor halt
# pick file 
file build/uart.elf
# Load it up
load
