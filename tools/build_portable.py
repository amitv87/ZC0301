#!/usr/bin/env python3
"""Build a single self-contained executable of the camera tool.

The result needs nothing on the target machine: no Python, no libusb, no pip.
Run this on the OS you want to build for — PyInstaller cannot cross-compile.

    python3 -m pip install pyinstaller libusb1
    python3 tools/build_portable.py

The one fiddly part is libusb. python-libusb1 looks for the library inside its
own package directory first, so we stage a correctly named copy there for
PyInstaller's hook to collect. On Windows the pip wheel already ships
`usb1/libusb-1.0.dll`, so there is nothing to do.
"""

import argparse
import glob
import os
import platform
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = 'zc3xx-camera'

# what python-libusb1 looks for inside its package directory, per platform
WANTED = {
    'Windows': 'libusb-1.0.dll',
    'Darwin': 'libusb-1.0.dylib',
    'Linux': 'libusb-1.0.so.0',
}

# where the system copy usually lives, when it is not already bundled
CANDIDATES = {
    'Darwin': [
        '/opt/homebrew/lib/libusb-1.0.0.dylib',        # Apple silicon brew
        '/usr/local/lib/libusb-1.0.0.dylib',           # Intel brew
        '/opt/local/lib/libusb-1.0.0.dylib',           # MacPorts
    ],
    'Linux': [
        '/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0',
        '/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0',
        '/usr/lib64/libusb-1.0.so.0',
        '/usr/lib/libusb-1.0.so.0',
    ],
    'Windows': [],
}


def find_libusb(system):
    """Return a path to a libusb shared library, or None if pip already has it."""
    import usb1
    package_dir = os.path.dirname(usb1.__file__)

    # already inside the package? (the Windows wheels ship it)
    for name in ('libusb-1.0.dll', 'libusb-1.0.dylib',
                 'libusb-1.0.so.0', 'libusb-1.0.so'):
        if os.path.exists(os.path.join(package_dir, name)):
            print('libusb already bundled in the wheel: %s' % name)
            return None

    for path in CANDIDATES.get(system, []):
        if os.path.exists(path):
            return path

    # last resort: ask ctypes where the system copy is
    from ctypes.util import find_library
    found = find_library('usb-1.0')
    if found and os.path.isabs(found) and os.path.exists(found):
        return found

    raise SystemExit(
        'Could not find libusb-1.0. Install it (brew install libusb, '
        'apt install libusb-1.0-0, ...) or pass --libusb /path/to/library.')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--libusb', help='explicit path to the libusb library')
    parser.add_argument('--distpath', default=os.path.join(ROOT, 'dist'))
    parser.add_argument('--debug', action='store_true',
                        help='keep the PyInstaller build log')
    args = parser.parse_args()

    system = platform.system()
    if system not in WANTED:
        raise SystemExit('unsupported platform: %s' % system)

    staged = tempfile.mkdtemp(prefix='zc3xx-build-')
    try:
        command = [
            sys.executable, '-m', 'PyInstaller',
            '--onefile', '--noconfirm', '--clean',
            '--name', NAME,
            '--distpath', args.distpath,
            '--workpath', os.path.join(staged, 'build'),
            '--specpath', staged,
            '--add-data', os.path.join(ROOT, 'webusb') + os.pathsep + 'webusb',
        ]

        source = args.libusb or find_libusb(system)
        if source:
            # PyInstaller keeps the basename, and the loader wants an exact
            # name, so stage a renamed copy
            target = os.path.join(staged, WANTED[system])
            shutil.copy2(source, target)
            print('bundling %s as usb1/%s' % (source, WANTED[system]))
            command += ['--add-binary', target + os.pathsep + 'usb1']

        command.append(os.path.join(ROOT, 'zc3xx_capture.py'))

        print('$ ' + ' '.join(command))
        result = subprocess.run(command, cwd=ROOT)
        if result.returncode:
            return result.returncode
    finally:
        if not args.debug:
            shutil.rmtree(staged, ignore_errors=True)

    produced = glob.glob(os.path.join(args.distpath, NAME + '*'))
    for path in produced:
        print('\nbuilt %s (%.1f MB)' % (path, os.path.getsize(path) / 1e6))
    print('\nSmoke test it with:  %s probe' % (produced[0] if produced else NAME))
    return 0


if __name__ == '__main__':
    sys.exit(main())
