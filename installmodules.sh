echo "Mounting /system as rw"
adb remount

echo "Pushing Wifi Module"
adb push ./drivers/net/wireless/bcm4329_248/bcm4329.ko /system/lib/modules/
echo "Done Pushing Wifi Module"

echo "Pushing Wimax Modules"
adb push ./drivers/net/wimax/SQN/sequans_sdio.ko /system/lib/modules/
adb push ./drivers/net/wimax/wimaxdbg/wimaxdbg.ko /system/lib/modules/
adb push ./drivers/net/wimax/wimaxuart/wimaxuart.ko /system/lib/modules/
echo "Done Pushing Wimax Modules"

echo "Setting Permissions"
adb shell chmod 644 /system/lib/modules/bcm4329.ko
adb shell chmod 644 /system/lib/modules/sequans_sdio.ko
adb shell chmod 644 /system/lib/modules/wimaxdbg.ko
adb shell chmod 644 /system/lib/modules/wimaxuart.ko
echo "Done Setting Permissions"