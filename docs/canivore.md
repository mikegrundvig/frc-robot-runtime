# A CANivore on Systemcore

Systemcore has no driver for a CANivore until CTRE's `canivore-usb` package is installed on it.
Without it, the robot program starts, and every device on the CANivore fails to answer. This page
installs it and says how the robot tells you it's working. It's for a robot whose wiring puts
devices on a CANivore (`CanBus.canivore(...)`), with the CANivore's USB cable in Systemcore.

Facts here are from WPILib's [SystemcoreTesting](https://github.com/wpilibsuite/SystemcoreTesting)
pages for the alpha software the template is on, and are unconfirmed on a production Systemcore.

## Install the package, once per Systemcore

1. **Download the package for your Systemcore's image.** WPILib's
   [CTR-Phoenix page](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md)
   lists the `canivore-usb` `.ipk` files for each WPILib release, one per Systemcore image. For
   WPILib alpha 7 (Phoenix 6 26.70.0-alpha-2, the template's) it's version 1.18, one file: pick the
   one the page names for your image. Each WPILib release that moves Phoenix may bring its own.
2. **Open Systemcore's web page:** `http://robot.local`, over USB-C, Ethernet or Wi-Fi.
3. **Add Package:** choose the `.ipk`, and install it.
4. **Deploy the robot program and look at its alerts.** No `CAN/canivore/<name>` alert means
   Phoenix sees the CANivore.

## What the robot says

On a robot, the program asks Phoenix about each CANivore its wiring names, once a second. One
Phoenix doesn't see raises a high alert, `CAN/canivore/<name>`, that names it, how many devices are
on it, and Phoenix's own reason, and clears once Phoenix sees it again:

```text
The CANivore drive isn't seen, so its 13 devices don't answer: check its USB cable and its CAN
power, and that CTRE's canivore-usb package is installed (docs/canivore.md) (CANNotFound: ...)
```

So a CANivore that's unplugged, unpowered, renamed in Tuner X, or has no driver shows as one alert,
not as every module's `DriveConnected` false. On a laptop driving real devices
([bench.md](bench.md)), the same alert says to check that Tuner X sees it.

## After a Systemcore update

Not yet known: whether an image update keeps installed packages. Systemcore's update page asks
what to keep, and WPILib recommends keeping everything. If the alert appears after an update,
install the package again, the `.ipk` for the new image.
