"""Run once ON MaixCAM Lite via MaixVision, then reconnect power manually.

Enable only the UVC function. Existing USB network settings remain unchanged.
Reference: MaixPy examples/tools/maixcam_switch_usb_mode.py (MaixCAM family).
The SDK ID selects the image's USB configuration layout, not its screen variant.
"""
import os


def main():
    from maix import sys as device_sys

    board = device_sys.device_id()
    print("Device:", board, flush=True)
    if board != "maixcam":
        raise RuntimeError("This MaixCAM Lite setup uses the first-generation MaixCAM "
                           "USB layout (SDK ID 'maixcam'). Report the printed device "
                           "ID and image version before using a different layout.")
    if not os.path.isfile("/usr/bin/usb_util"):
        raise RuntimeError("usb_util is missing; check the MaixCAM system/runtime version.")
    if os.path.exists("/boot/usb.host"):
        raise RuntimeError("USB is configured as HOST. Check the connection mode first.")
    if not os.path.exists("/boot/usb.dev"):
        raise RuntimeError("Expected /boot/usb.dev for this MaixCAM image. "
                           "No configuration was changed.")
    path = "/boot/usb.uvc"
    if os.path.exists(path):
        print("UVC configuration is already enabled.", flush=True)
    else:
        with open(path, "x"):
            pass
        os.sync()
        print("Enabled UVC. Existing RNDIS/NCM settings were preserved.", flush=True)
    print("Stop this script, reconnect USB power, wait for boot, reconnect "
          "MaixVision, then run stream_uvc.py. USB is NOT restarted by this script.",
          flush=True)


if __name__ == "__main__":
    main()
