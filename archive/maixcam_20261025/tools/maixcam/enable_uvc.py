"""Run once ON MaixCAM via MaixVision, then reconnect power manually.

Enable only the UVC function. Existing USB network settings remain unchanged.
Reference: MaixPy examples/tools/maixcam_switch_usb_mode.py (legacy MaixCAM).
"""
import os


def main():
    from maix import sys as device_sys

    board = device_sys.device_id()
    print("Device:", board, flush=True)
    if board != "maixcam":
        raise RuntimeError("This setup is verified from legacy MaixCAM source; "
                           "report the printed device ID before configuring another model.")
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
