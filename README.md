# ZC0301 "PC Camera" userspace driver

Brings up a driverless Vimicro / Z-Star **ZC0301(P)** USB webcam
(`0ac8:301b`, and its siblings) from userspace — once in Python over libusb,
once in the browser over **WebUSB**.

```
[USB] Bus[1].Addr[1]: vid 0x0ac8, pid 0x301b, USB1.1 @ 12Mbps (FULL speed)
[USB]   Manufacturer: Vimicro Corp.      Product: PC Camera
[USB]     Interface[0] alt 0..7: VENDOR_SPEC FF.FF.FF
[USB]       Endpoint[0]: 0x81 IN ISOCHRONOUS 0..896 bytes
[USB]       Endpoint[1]: 0x82 IN INTERRUPT 8 bytes
```

There is no UVC here: the interface is vendor-specific, so the bridge and its
image sensor are driven entirely through vendor control transfers, and the
video arrives on an isochronous endpoint. Linux supports these cameras with the
gspca `zc3xx` driver; this repo is a port of that driver's logic.

## How the camera works

* **Bridge registers** — vendor request `0xa0` writes one byte
  (`wValue` = data, `wIndex` = register), `0xa1` reads one byte back.
* **Sensor registers** — the bridge has an I2C/3-wire master behind registers
  `0x90`–`0x96`. The sensor is not identified anywhere in the descriptors, so
  it has to be *probed*: poke each candidate's ID registers and see what
  answers. On this camera that lands on a **Hynix HV7131R**.
* **Start-up** — each sensor has a table of a few dozen register writes for
  full size and another for half size (the difference is bit 4 of
  `R002_CLOCKSELECT`, which turns on the bridge's 2:1 scaler), followed by a
  colour matrix, a 16-point gamma curve and its gradient, sharpness, and the
  JPEG quality step.
* **Video** — the bridge emits JPEG with the quantisation and Huffman tables
  *stripped out*. Each frame starts with a proprietary 18-byte header

  ```
  ff d8 ff fe 00 0e 00 00 ss ss 00 01 ww ww hh hh pp pp
          |                 |           |     |     '- packet sequence
          |                 |           |     '------- height
          |                 |           '------------- width
          |                 '------------------------- frame sequence
          '--------------------------- a 14-byte JPEG COM segment
  ```

  and ends with `ff d9` plus one trailing byte. Both drivers strip that header,
  synthesise a normal JFIF header for the configured size and quality, and glue
  it in front.
* **Bit-rate control** — 640×480 does not fit in USB 1.1 bandwidth at full
  rate. Bit 0 of register `0x11` flags a FIFO overflow; register `0x07` caps
  the bytes per isochronous packet. Both drivers run the kernel's feedback
  loop: tighten on overflow, relax after ten clean polls.

## Layout

| path | what it is |
| --- | --- |
| `ref/` | the Linux gspca sources this is ported from (reference only) |
| `tools/gen_tables.py` | parses `ref/zc3xx.c` and emits the register tables |
| `zc3xx/_tables.py`, `webusb/zc3xx-tables.js` | **generated** — do not edit |
| `zc3xx/camera.py` | the driver: probe, start-up, controls, frame assembly |
| `zc3xx_capture.py` | CLI: `probe`, `snap`, `stream`, `webusb` |
| `webusb/zc3xx.js` | the same driver against the WebUSB API |
| `webusb/index.html` | a viewer page for it |
| `tools/build_portable.py` | freezes it all into one dependency-free binary |
| `.github/workflows/build.yml` | builds that binary for all four targets |
| `packaging/99-zc3xx.rules` | udev rule for non-root access on Linux |

The 123 register tables (4860 entries) are extracted from the kernel source
rather than transcribed, so both drivers are provably identical to the kernel's
on that data — `tools/gen_tables.py` regenerates both at once.

## Python / libusb

Needs libusb and the `libusb1` bindings:

```sh
brew install libusb          # or your platform's equivalent
python3 -m pip install --user libusb1
```

```sh
python3 zc3xx_capture.py probe            # identify bridge + sensor
python3 zc3xx_capture.py snap -n 5        # save frame000.jpg …
python3 zc3xx_capture.py stream           # MJPEG at http://127.0.0.1:8088/
```

Useful flags: `--mode 1` for 640×480, `--quality 50|75|87`,
`--light-frequency 1|2` for the 50/60 Hz flicker filter, `--no-autogain`,
`--gamma`, `--brightness`, `--contrast`, `--sharpness`, `--debug`.

On macOS and Linux no kernel driver claims a vendor-specific interface, so this
runs without root.

On **Windows**, bind the device to **WinUSB** with [Zadig](https://zadig.akeo.ie/)
first, then `pip install libusb1` — its Windows wheels bundle a `libusb-1.0.dll`
new enough to do isochronous transfers. This is the supported path on Windows,
since the WebUSB page cannot stream there. If the transfers fail to start, the
camera is full speed and alt setting 7 claims 896 of the 1023 bytes available
per frame, so try `--alt 5` or a port on a different controller.

Measured on the `0ac8:301b` this was developed against (alt 7, 896 bytes per
isochronous packet, quality 75): **28.8 fps at 320x240** and **14.4 fps at
640x480**, no dropped frames at either size.

## WebUSB

WebUSB needs a secure context, so serve the page over `localhost`:

```sh
python3 zc3xx_capture.py webusb           # http://127.0.0.1:8089/
```

Open it in Chrome or Edge, click **Connect camera**, pick the device, then
**Start**. Probing takes a few seconds — every register access is a separate
round trip through the browser.

Video needs **isochronous transfers** (`isochronousTransferIn`). Firefox and
Safari do not implement WebUSB at all, and this device has no bulk endpoint to
fall back to, so Chromium is the only option — and only on some platforms:

| platform | control transfers (probe) | isochronous (video) |
| --- | --- | --- |
| macOS | yes | **yes** — verified, Chrome 150 / macOS 12, Apple silicon |
| Linux, ChromeOS | yes | yes |
| Windows | yes | **no** — see below |

The libusb driver has no such gap: it is verified streaming on **macOS** and on
**Windows** (WinUSB, from the frozen one-file build).

### Windows cannot stream over WebUSB

Chromium's Windows USB backend never implemented isochronous transfers. The
entire body of `UsbDeviceHandleWin::IsochronousTransferIn` in
`services/device/usb/usb_device_handle_win.cc` is:

```cc
// Isochronous is not yet supported on Windows.
ReportIsochronousError(packet_lengths, std::move(callback),
                       UsbTransferStatus::TRANSFER_ERROR);
```

blink turns that into `NetworkError: A transfer error has occurred.` The device
still enumerates and the sensor still probes correctly, because control
transfers take a different path — only the video fails, which makes it look
like a configuration problem. It is not: no driver swap, alternate setting or
transfer size changes it. The page detects Windows and says so before you press
Start.

Use the libusb driver there instead. `libusb` has supported isochronous on
WinUSB since 1.0.23, and the standalone Windows build is verified working
against real hardware.

### Other notes

* WebUSB refuses to claim interfaces with a protected class (audio, HID, mass
  storage, **video**, …). This camera is class `FF`, so it is allowed — a
  proper UVC webcam would not be.
* Only one process can hold the interface. Stop the Python driver before using
  the page, and vice versa.
* The page keeps six isochronous transfers of 32 packets in flight and retires
  them in submission order, so packets reach the frame assembler in the order
  the bridge sent them.

## Distributing it

`tools/build_portable.py` freezes the tool into **one executable with no
runtime dependencies** — no Python, no pip, no libusb on the target machine.
The Windows build produced this way is verified streaming from a real camera:

```sh
python3 -m pip install pyinstaller libusb1
python3 tools/build_portable.py            # -> dist/zc3xx-camera (~4 MB)
```

The awkward part is libusb, and the script handles it: on Windows the pip wheel
already ships `usb1/libusb-1.0.dll`, and elsewhere it stages a correctly named
copy of the system library where python-libusb1 will look for it first.

PyInstaller cannot cross-compile, so each platform's binary has to be built on
that platform. `.github/workflows/build.yml` does all four on GitHub's free
runners and attaches them to a release when you push a `v*` tag — Linux,
macOS arm64, macOS x86_64 and Windows, at no cost for a public repo.

What packaging **cannot** remove, on any of these:

* **Windows** — the device still has to be bound to WinUSB with
  [Zadig](https://zadig.akeo.ie/) once, as administrator. A vendor-specific
  device with no driver is unreachable otherwise. Shipping a signed driver
  package instead would need a code-signing certificate.
* **macOS** — an unsigned binary is quarantined by Gatekeeper. Users can
  right-click → Open, or `xattr -dr com.apple.quarantine zc3xx-camera`. Making
  the warning go away entirely needs an Apple Developer ID and notarisation
  (currently $99/year).
* **Linux** — either run as root or install `packaging/99-zc3xx.rules`.

None of that costs anything if you accept the one-time warnings; money only
buys them away.

If you only care about macOS, Linux and ChromeOS, the cheapest distribution is
no binary at all: publish `webusb/` on GitHub Pages and hand out the URL. It is
served over HTTPS, so WebUSB works, and there is nothing to install or update.
That does not help Windows, for the reason above.

## Licence

The register tables and the driver logic are derived from the Linux gspca
`zc3xx` driver, © Jean-François Moine and Michel Xhaard, **GPL-2.0-or-later**,
which that derivation carries over to this code.
