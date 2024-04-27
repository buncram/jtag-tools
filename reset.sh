if [ ! -d /sys/class/gpio/gpio21 ]
then
    echo "21" > /sys/class/gpio/export
fi

echo "out" > /sys/class/gpio/gpio21/direction

echo 0 > /sys/class/gpio/gpio21/value

sudo killall openocd
sudo openocd -f jlink.cfg -f cramsoc.cfg &
sleep 0.5

echo 1 > /sys/class/gpio/gpio21/value

# riscv-none-elf-gdb -ex "target extended-remote :3333" -ex "mon reset" -ex "continue"
