#!/usr/bin/env python3
"""Bring up a Z-Star/Vimicro ZC0301 "PC Camera" (0x0ac8:0x301b) over libusb.

    python3 zc3xx_capture.py probe              # identify the sensor
    python3 zc3xx_capture.py snap -n 5          # save frames as JPEG files
    python3 zc3xx_capture.py stream             # MJPEG server on localhost
    python3 zc3xx_capture.py webusb             # serve the WebUSB page
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import usb1                                                  # noqa: E402

from zc3xx import ZC3xxError, find_devices, open_camera      # noqa: E402
from zc3xx import _tables as T                               # noqa: E402

def _resource_dir():
    """Where our data files live, whether running from source or frozen.

    PyInstaller unpacks a one-file build into a temporary directory and points
    sys._MEIPASS at it.
    """
    return getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))


HERE = _resource_dir()


def add_camera_args(parser):
    parser.add_argument('--mode', type=int, default=0,
                        help='0 = half size (default), 1 = full size')
    parser.add_argument('--quality', type=int, default=None,
                        help='JPEG quality; snaps to 50, 75 or 87')
    parser.add_argument('--alt', type=int, default=None,
                        help='isochronous alternate setting (default: largest)')
    parser.add_argument('--brightness', type=int, default=None)
    parser.add_argument('--contrast', type=int, default=None)
    parser.add_argument('--gamma', type=int, default=None, choices=range(1, 7))
    parser.add_argument('--sharpness', type=int, default=None,
                        choices=range(0, 4))
    parser.add_argument('--no-autogain', action='store_true')
    parser.add_argument('--light-frequency', type=int, default=0,
                        choices=(0, 1, 2),
                        help='0 = off (default), 1 = 50Hz, 2 = 60Hz')
    parser.add_argument('--force-sensor', default=None,
                        help='skip detection, e.g. SENSOR_PAS106')
    parser.add_argument('--debug', action='store_true')


def configure(camera, args):
    if args.brightness is not None:
        camera.brightness = args.brightness
    if args.contrast is not None:
        camera.contrast = args.contrast
    if args.gamma is not None:
        camera.gamma = args.gamma
    if args.sharpness is not None:
        camera.sharpness = args.sharpness
    if args.no_autogain:
        camera.autogain = 0
    camera.light_frequency = args.light_frequency


# ---------------------------------------------------------------------------
def cmd_probe(args):
    with usb1.USBContext() as context:
        devices = list(find_devices(context))
        if not devices:
            print('no ZC3xx camera found')
            return 1
        for device in devices:
            print('%04x:%04x on bus %d device %d'
                  % (device.getVendorID(), device.getProductID(),
                     device.getBusNumber(), device.getDeviceAddress()))

    with open_camera(debug=args.debug,
                     force_sensor=args.force_sensor) as camera:
        print('bridge:        ZC30%d' % (1 if camera.bridge == 0 else 3))
        print('sensor:        %s' % camera.sensor)
        if camera.chip_revision:
            print('chip revision: 0x%04x' % camera.chip_revision)
        print('modes:         %s'
              % ', '.join('%d = %dx%d' % (i, w, h)
                          for i, (w, h, _) in enumerate(camera.modes)))
        print('jpeg quality:  %d (reg08 = 0x%02x)'
              % (camera.quality, camera.reg08))
        print('defaults:      gamma %d, sharpness %d, brightness %d, '
              'contrast %d' % (camera.gamma, camera.sharpness,
                               camera.brightness, camera.contrast))
        if camera.exposure is not None:
            print('exposure:      0x%04x' % camera.exposure)
        # the bridge was just reset, so a read can legitimately time out
        dump = []
        for reg in (0x0002, 0x0008, 0x0010, 0x0011, 0x0180):
            try:
                dump.append('%04x=%02x' % (reg, camera.reg_r(reg)))
            except usb1.USBError as error:
                dump.append('%04x=%s' % (reg, error.__class__.__name__))
        print('registers:     ' + ' '.join(dump))
    return 0


def _run(camera, context, stop_predicate, timeout):
    """Pump libusb events plus the bridge's bit-rate-control loop."""
    deadline = time.time() + timeout
    while not stop_predicate() and time.time() < deadline:
        context.handleEventsTimeout(0.05)
        camera.poll()
    return stop_predicate()


def cmd_snap(args):
    frames = []
    with usb1.USBContext() as context:
        with open_camera(context, debug=args.debug,
                         force_sensor=args.force_sensor) as camera:
            configure(camera, args)
            width, height = camera.start(
                mode=args.mode, alt=args.alt, quality=args.quality,
                on_frame=frames.append)
            print('streaming %dx%d, sensor %s, quality %d'
                  % (width, height, camera.sensor, camera.quality))
            # the first frames after start-up are usually partial
            ok = _run(camera, context,
                      lambda: len(frames) >= args.count + args.skip,
                      args.timeout)
            if not ok:
                print('timed out after %.1fs with %d frame(s)'
                      % (args.timeout, len(frames)), file=sys.stderr)

    keep = frames[args.skip:args.skip + args.count]
    for i, frame in enumerate(keep):
        path = '%s%03d.jpg' % (args.output, i)
        with open(path, 'wb') as handle:
            handle.write(frame)
        print('wrote %s (%d bytes)' % (path, len(frame)))
    return 0 if keep else 1


# ---------------------------------------------------------------------------
class _FrameBroker:
    """Hands the newest JPEG frame to any number of HTTP clients."""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None
        self._serial = 0
        self.count = 0

    def publish(self, frame):
        with self._condition:
            self._frame = frame
            self._serial += 1
            self.count += 1
            self._condition.notify_all()

    def wait(self, last_serial, timeout=5.0):
        with self._condition:
            if self._serial == last_serial:
                self._condition.wait(timeout)
            return self._frame, self._serial


VIEWER_HTML = b"""<!doctype html>
<meta charset="utf-8"><title>ZC0301 PC Camera</title>
<style>
  :root { color-scheme: light dark; --bg:#f6f6f7; --fg:#16161a; --mut:#6b6b76; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#131316; --fg:#f2f2f4; --mut:#9a9aa6; }
  }
  body { background:var(--bg); color:var(--fg); margin:0; padding:24px 16px;
         font:15px/1.5 ui-sans-serif,-apple-system,Segoe UI,sans-serif;
         display:flex; flex-direction:column; align-items:center; gap:14px; }
  h1 { font-size:17px; font-weight:600; margin:0; }
  img { max-width:100%; image-rendering:pixelated; border-radius:8px;
        box-shadow:0 1px 3px rgba(0,0,0,.25); background:#000; }
  p { color:var(--mut); margin:0; font-size:13px; }
</style>
<h1>ZC0301 PC Camera</h1>
<img src="/stream.mjpg" alt="live camera stream">
<p>Served by zc3xx_capture.py over libusb.</p>
"""


def _make_handler(broker, status):
    from http.server import BaseHTTPRequestHandler

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def log_message(self, *_args):
            pass

        def do_GET(self):                                     # noqa: N802
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(VIEWER_HTML)))
                self.end_headers()
                self.wfile.write(VIEWER_HTML)
            elif self.path == '/status':
                body = ('{"frames": %d, "sensor": "%s", "size": "%dx%d"}'
                        % (broker.count, status['sensor'],
                           status['width'], status['height'])).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/stream.mjpg':
                self._stream()
            else:
                self.send_error(404)

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=zcframe')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            serial = -1
            try:
                while True:
                    frame, serial = broker.wait(serial)
                    if frame is None:
                        continue
                    self.wfile.write(b'--zcframe\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: %d\r\n\r\n' % len(frame))
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def cmd_stream(args):
    from http.server import ThreadingHTTPServer

    broker = _FrameBroker()
    status = {'sensor': '?', 'width': 0, 'height': 0}
    server = ThreadingHTTPServer((args.host, args.port),
                                 _make_handler(broker, status))
    thread = threading.Thread(target=server.serve_forever, daemon=True)

    with usb1.USBContext() as context:
        with open_camera(context, debug=args.debug,
                         force_sensor=args.force_sensor) as camera:
            configure(camera, args)
            width, height = camera.start(
                mode=args.mode, alt=args.alt, quality=args.quality,
                on_frame=broker.publish)
            status.update(sensor=camera.sensor, width=width, height=height)
            thread.start()
            print('sensor %s, %dx%d, quality %d'
                  % (camera.sensor, width, height, camera.quality))
            print('open http://%s:%d/  (ctrl-c to stop)'
                  % (args.host, args.port))
            last = time.time()
            shown = 0
            try:
                while True:
                    context.handleEventsTimeout(0.05)
                    camera.poll()
                    now = time.time()
                    if now - last >= 2.0:
                        fps = (broker.count - shown) / (now - last)
                        print('\r%d frames, %.1f fps, dropped %d   '
                              % (broker.count, fps, camera.frames_dropped),
                              end='', flush=True)
                        shown, last = broker.count, now
            except KeyboardInterrupt:
                print('\nstopping')
    server.shutdown()
    return 0


def cmd_webusb(args):
    """Serve webusb/ over http://localhost, which counts as a secure context."""
    import functools
    from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

    root = os.path.join(HERE, 'webusb')
    handler = functools.partial(SimpleHTTPRequestHandler, directory=root)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print('serving %s at http://%s:%d/' % (root, args.host, args.port))
    print('Chrome only; the camera must not be claimed by another process.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopping')
    return 0


def cmd_tables(args):
    print('%d usb_action tables generated from the gspca driver'
          % len(T.ACTIONS))
    for name in sorted(T.ACTIONS):
        if args.filter and args.filter not in name:
            continue
        print('  %-28s %4d entries' % (name, len(T.ACTIONS[name])))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)

    probe = sub.add_parser('probe', help='identify the bridge and sensor')
    add_camera_args(probe)
    probe.set_defaults(func=cmd_probe)

    snap = sub.add_parser('snap', help='save frames as JPEG files')
    add_camera_args(snap)
    snap.add_argument('-n', '--count', type=int, default=1)
    snap.add_argument('-o', '--output', default='frame')
    snap.add_argument('--skip', type=int, default=2,
                      help='discard this many frames first (default 2)')
    snap.add_argument('--timeout', type=float, default=15.0)
    snap.set_defaults(func=cmd_snap)

    stream = sub.add_parser('stream', help='serve MJPEG over HTTP')
    add_camera_args(stream)
    stream.add_argument('--host', default='127.0.0.1')
    stream.add_argument('--port', type=int, default=8088)
    stream.set_defaults(func=cmd_stream)

    webusb = sub.add_parser('webusb', help='serve the WebUSB page')
    webusb.add_argument('--host', default='127.0.0.1')
    webusb.add_argument('--port', type=int, default=8089)
    webusb.set_defaults(func=cmd_webusb)

    tables = sub.add_parser('tables', help='list the generated init tables')
    tables.add_argument('filter', nargs='?')
    tables.set_defaults(func=cmd_tables)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except ZC3xxError as error:
        print('error: %s' % error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
