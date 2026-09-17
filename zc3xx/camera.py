"""Userspace driver for the Z-Star/Vimicro ZC0301(P) bridge (0x0ac8:0x301b & co).

This is a port of the Linux gspca `zc3xx` kernel driver to libusb.  The
register/init tables live in `_tables.py` and are generated straight from the
kernel sources by `tools/gen_tables.py`; everything here is the control flow
around them (sensor probing, start-up sequence, image controls and the
isochronous frame assembler).

The bridge hands us JPEG data with the tables stripped out and an 18-byte
proprietary header on the first packet of each frame, so we rebuild a normal
JFIF header (`jpeg_header`) and glue it in front.
"""

import contextlib
import time

import usb1

from . import _tables as T

VENDOR_ID = 0x0ac8
PRODUCT_IDS = (0x0301, 0x0302, 0x301b, 0x303b, 0x305b, 0x307b)

#: bridge reg08 default: bits 1-2 select the JPEG quality (3 -> 75%)
REG08_DEF = 3

BRIDGE_ZC301 = 0
BRIDGE_ZC303 = 1

#: video interface (the only one these cameras expose)
IFACE = 0
#: isochronous video endpoint / interrupt "button" endpoint
EP_VIDEO = 0x81
EP_BUTTON = 0x82


class ZC3xxError(Exception):
    pass


# ---------------------------------------------------------------------------
# JPEG header synthesis (port of gspca's jpeg.h)
# ---------------------------------------------------------------------------
def jpeg_header(width, height, quality=75, samples_y=0x21):
    """Build the JFIF header the bridge omits from its stream."""
    hdr = bytearray(T.JPEG_HEAD)

    off = T.JPEG_HEIGHT_OFFSET
    hdr[off + 0] = (height >> 8) & 0xff
    hdr[off + 1] = height & 0xff
    hdr[off + 2] = (width >> 8) & 0xff
    hdr[off + 3] = width & 0xff
    hdr[off + 6] = samples_y

    if quality <= 0:
        scale = 5000
    elif quality < 50:
        scale = 5000 // quality
    else:
        scale = 200 - quality * 2
    for i in range(64):
        for base in (T.JPEG_QT0_OFFSET, T.JPEG_QT1_OFFSET):
            val = (T.JPEG_HEAD[base + i] * scale + 50) // 100
            hdr[base + i] = min(255, max(1, val))
    return bytes(hdr)


# ---------------------------------------------------------------------------
# frame assembly (port of zc3xx's sd_pkt_scan)
# ---------------------------------------------------------------------------
class FrameAssembler:
    """Turns isochronous packets into complete JPEG frames.

    The bridge starts every frame with

        ff d8 ff fe 00 0e 00 00 ss ss 00 01 ww ww hh hh pp pp

    (a JPEG SOI plus an 18-byte COM segment holding the frame sequence number,
    the window dimensions and the packet sequence number) and ends it with
    ff d9 plus one trailing byte the kernel driver also discards.
    """

    def __init__(self, jpeg_hdr):
        self.jpeg_hdr = jpeg_hdr
        self._buf = bytearray()
        self._in_frame = False
        self.frames_started = 0
        self.frames_dropped = 0
        self.last_header = None

    def feed(self, data):
        """Consume one isochronous packet, yielding any completed frame."""
        if not data:
            return None

        # end of frame?
        if len(data) >= 3 and data[-3] == 0xff and data[-2] == 0xd9:
            if not self._in_frame:
                return None
            self._buf += data[:-1]
            frame = bytes(self._buf)
            self._buf = bytearray()
            self._in_frame = False
            return frame

        # start of frame?
        if len(data) >= 2 and data[0] == 0xff and data[1] == 0xd8:
            if self._in_frame:
                self.frames_dropped += 1
            self.last_header = self._parse_header(data)
            self._buf = bytearray(self.jpeg_hdr)
            self._in_frame = True
            self.frames_started += 1
            data = data[18:]

        if self._in_frame:
            self._buf += data
        return None

    @staticmethod
    def _parse_header(data):
        if len(data) < 18:
            return None
        return {
            'sequence': (data[8] << 8) | data[9],
            'width': (data[12] << 8) | data[13],
            'height': (data[14] << 8) | data[15],
            'packet': (data[16] << 8) | data[17],
        }


# ---------------------------------------------------------------------------
# the camera
# ---------------------------------------------------------------------------
class Camera:
    """A ZC0301/ZC0302 camera opened over libusb."""

    def __init__(self, handle, product_id=0x301b, debug=False, context=None):
        self.handle = handle
        self.context = context
        self.product_id = product_id
        self.debug = debug

        self.bridge = BRIDGE_ZC301 if product_id == 0x301b else BRIDGE_ZC303
        # gspca seeds sd->sensor from driver_info, which is 0 (== ADCM2700)
        # for the auto-probed models.
        self.sensor = T.SENSORS[0]
        self.chip_revision = 0
        self.reg08 = REG08_DEF

        self.modes = []
        self.mode = 0
        self.width = 0
        self.height = 0
        self.jpeg_hdr = b''

        # image controls, defaults straight from sd_init_controls()
        self.brightness = 128
        self.contrast = 128
        self.gamma = 4
        self.sharpness = 2
        self.autogain = 1
        self.exposure = None
        self.light_frequency = 0        # 0 = disabled, 1 = 50Hz, 2 = 60Hz

        self.streaming = False
        self._alt = 0
        self._transfers = []
        self._assembler = None
        self._on_frame = None
        # bit-rate-control state, mirrors the driver's transfer_update() thread
        self._reg07 = 0
        self._brc_good = 0
        self._brc_next = 0.0

    # -- plumbing ----------------------------------------------------------
    def _log(self, fmt, *args):
        if self.debug:
            print('[zc3xx] ' + (fmt % args if args else fmt))

    def reg_w(self, value, index):
        """Write one bridge register (vendor request 0xa0)."""
        self.handle.controlWrite(0x40, 0xa0, value, index, b'', timeout=500)

    def reg_r(self, index):
        """Read one bridge register (vendor request 0xa1)."""
        data = self.handle.controlRead(0xc0, 0xa1, 0x01, index, 1, timeout=500)
        return data[0]

    def i2c_write(self, reg, val_lo, val_hi):
        """Write a sensor register through the bridge's I2C/3-wire master."""
        self.reg_w(reg, 0x0092)
        self.reg_w(val_lo, 0x0093)
        self.reg_w(val_hi, 0x0094)
        self.reg_w(0x01, 0x0090)            # write command
        time.sleep(0.001)
        status = self.reg_r(0x0091)
        if status != 0x00:
            self._log('i2c_w status error %02x', status)
        return status

    def i2c_read(self, reg):
        """Read a 16-bit sensor register through the bridge."""
        self.reg_w(reg, 0x0092)
        self.reg_w(0x02, 0x0090)            # read command
        time.sleep(0.020)
        status = self.reg_r(0x0091)
        if status != 0x00:
            self._log('i2c_r status error %02x', status)
        return self.reg_r(0x0095) | (self.reg_r(0x0096) << 8)

    def usb_exchange(self, actions):
        """Replay one of the driver's usb_action tables."""
        if isinstance(actions, str):
            actions = T.ACTIONS[actions]
        for req, val, idx in actions:
            if req == 0xa0:
                self.reg_w(val, idx)
            elif req == 0xa1:
                self.reg_r(idx)
            elif req == 0xaa:
                self.i2c_write(val, idx & 0xff, idx >> 8)
            elif req == 0xbb:
                self.i2c_write(idx >> 8, idx & 0xff, val)
            else:                            # 0xdd: delay
                time.sleep(idx / 1000.0)
            time.sleep(0.001)

    # -- sensor probing (vga_2wr_probe / vga_3wr_probe / sif_probe) ---------
    def _start_2wr_probe(self, sensor):
        self.reg_w(0x01, 0x0000)
        self.reg_w(sensor, 0x0010)
        self.reg_w(0x01, 0x0001)
        self.reg_w(0x03, 0x0012)
        self.reg_w(0x01, 0x0012)

    def _send_unknown(self, sensor):
        self.reg_w(0x01, 0x0000)            # bridge reset
        if sensor == 'SENSOR_PAS106':
            self.reg_w(0x03, 0x003a)
            self.reg_w(0x0c, 0x003b)
            self.reg_w(0x08, 0x0038)
        elif sensor in ('SENSOR_ADCM2700', 'SENSOR_GC0305', 'SENSOR_OV7620',
                        'SENSOR_MT9V111_1', 'SENSOR_MT9V111_3',
                        'SENSOR_PB0330', 'SENSOR_PO2030'):
            self.reg_w(0x0d, 0x003a)
            self.reg_w(0x02, 0x003b)
            self.reg_w(0x00, 0x0038)
        elif sensor in ('SENSOR_HV7131R', 'SENSOR_PAS202B'):
            self.reg_w(0x03, 0x003b)
            self.reg_w(0x0c, 0x003a)
            self.reg_w(0x0b, 0x0039)
            if sensor == 'SENSOR_PAS202B':
                self.reg_w(0x0b, 0x0038)

    def _sif_probe(self):
        self._start_2wr_probe(0x0f)         # PAS106
        self.reg_w(0x08, 0x008d)
        time.sleep(0.150)
        check = (((self.i2c_read(0x00) & 0x0f) << 4)
                 | ((self.i2c_read(0x01) & 0xf0) >> 4))
        self._log('probe sif 0x%04x', check)
        if check == 0x0007:
            self._send_unknown('SENSOR_PAS106')
            return 0x0f
        return -1

    def _vga_2wr_probe(self):
        self._start_2wr_probe(0x00)         # HV7131B
        self.i2c_write(0x01, 0xaa, 0x00)
        if self.i2c_read(0x01) != 0:
            return 0x00

        self._start_2wr_probe(0x04)         # CS2102
        self.i2c_write(0x01, 0xaa, 0x00)
        if self.i2c_read(0x01) != 0:
            return 0x04

        self._start_2wr_probe(0x06)         # OmniVision
        self.reg_w(0x08, 0x008d)
        self.i2c_write(0x11, 0xaa, 0x00)
        omnivision = self.i2c_read(0x11) != 0

        if not omnivision:
            self._start_2wr_probe(0x08)     # HDCS2020
            self.i2c_write(0x1c, 0x00, 0x00)
            self.i2c_write(0x15, 0xaa, 0x00)
            if self.i2c_read(0x15) != 0:
                return 0x08

            self._start_2wr_probe(0x0a)     # PB0330
            self.i2c_write(0x07, 0xaa, 0xaa)
            if self.i2c_read(0x07) != 0:
                return 0x0a
            if self.i2c_read(0x03) != 0:
                return 0x0a
            if self.i2c_read(0x04) != 0:
                return 0x0a

            self._start_2wr_probe(0x0c)     # ICM105A
            self.i2c_write(0x01, 0x11, 0x00)
            if self.i2c_read(0x01) != 0:
                return 0x0c

            self._start_2wr_probe(0x0e)     # PAS202BCB
            self.reg_w(0x08, 0x008d)
            self.i2c_write(0x03, 0xaa, 0x00)
            time.sleep(0.050)
            if self.i2c_read(0x03) != 0:
                self._send_unknown('SENSOR_PAS202B')
                return 0x0e

            self._start_2wr_probe(0x02)     # TAS5130C
            self.i2c_write(0x01, 0xaa, 0x00)
            if self.i2c_read(0x01) != 0:
                return 0x02

        self.reg_r(0x0010)
        self.reg_r(0x0010)
        self.reg_w(0x01, 0x0000)
        self.reg_w(0x01, 0x0001)
        self.reg_w(0x06, 0x0010)            # OmniVision
        self.reg_w(0xa1, 0x008b)
        self.reg_w(0x08, 0x008d)
        time.sleep(0.500)
        self.reg_w(0x01, 0x0012)
        self.i2c_write(0x12, 0x80, 0x00)    # sensor reset
        val = (self.i2c_read(0x0a) << 8) | self.i2c_read(0x0b)
        self._log('probe 2wr ov vga 0x%04x', val)
        if val == 0x7631:                   # OV7630C
            self.reg_w(0x06, 0x0010)
        elif val not in (0x7620, 0x7648):
            return -1
        return val

    def _vga_3wr_probe(self):
        self.reg_w(0x02, 0x0010)
        self.reg_r(0x0010)
        self.reg_w(0x01, 0x0000)
        self.reg_w(0x00, 0x0010)
        self.reg_w(0x01, 0x0001)
        self.reg_w(0x91, 0x008b)
        self.reg_w(0x03, 0x0012)
        self.reg_w(0x01, 0x0012)
        self.reg_w(0x05, 0x0012)
        for reg in (0x14, 0x15, 0x16):
            if self.i2c_read(reg) != 0:
                return 0x11                 # HV7131R

        self.reg_w(0x02, 0x0010)
        val = (self.reg_r(0x000b) << 8) | self.reg_r(0x000a)
        self._log('probe 3wr vga 1 0x%04x', val)
        self.reg_r(0x0010)
        if (val & 0xff00) == 0x6400:
            return 0x02                     # TAS5130C
        for revision, internal_id in T.CHIP_REVISION_SENSOR:
            if revision == val:
                self.chip_revision = val
                self._send_unknown('SENSOR_PB0330')
                return internal_id

        self.reg_w(0x01, 0x0000)            # check PB0330
        self.reg_w(0x01, 0x0001)
        self.reg_w(0xdd, 0x008b)
        self.reg_w(0x0a, 0x0010)
        self.reg_w(0x03, 0x0012)
        self.reg_w(0x01, 0x0012)
        if self.i2c_read(0x00) != 0:
            self._log('probe 3wr vga type 0a')
            return 0x0a

        self.reg_w(0x01, 0x0000)            # check GC0303 / GC0305
        self.reg_w(0x01, 0x0001)
        self.reg_w(0x98, 0x008b)
        self.reg_w(0x01, 0x0010)
        self.reg_w(0x03, 0x0012)
        time.sleep(0.002)
        self.reg_w(0x01, 0x0012)
        val = self.i2c_read(0x00)
        if val != 0:
            self._log('probe 3wr vga type %02x', val)
            if val == 0x0011:
                return 0x0303               # GC0303
            if val == 0x0029:
                self._send_unknown('SENSOR_GC0305')
            return val

        self.reg_w(0x01, 0x0000)            # check OmniVision
        self.reg_w(0x01, 0x0001)
        self.reg_w(0xa1, 0x008b)
        self.reg_w(0x08, 0x008d)
        self.reg_w(0x06, 0x0010)
        self.reg_w(0x01, 0x0012)
        self.reg_w(0x05, 0x0012)
        if self.i2c_read(0x1c) == 0x007f and self.i2c_read(0x1d) == 0x00a2:
            self._send_unknown('SENSOR_OV7620')
            return 0x06

        self.reg_w(0x01, 0x0000)
        self.reg_w(0x00, 0x0002)
        self.reg_w(0x01, 0x0010)
        self.reg_w(0x01, 0x0001)
        self.reg_w(0xee, 0x008b)
        self.reg_w(0x03, 0x0012)
        self.reg_w(0x01, 0x0012)
        self.reg_w(0x05, 0x0012)
        val = (self.i2c_read(0x00) << 8) | self.i2c_read(0x01)
        self._log('probe 3wr vga 2 0x%04x', val)
        if val == 0x2030:
            self._log('sensor PO2030 rev 0x%02x', self.i2c_read(0x02))
            self._send_unknown('SENSOR_PO2030')
            return val

        self.reg_w(0x01, 0x0000)
        self.reg_w(0x0a, 0x0010)
        self.reg_w(0xd3, 0x008b)
        self.reg_w(0x01, 0x0001)
        self.reg_w(0x03, 0x0012)
        self.reg_w(0x01, 0x0012)
        self.reg_w(0x05, 0x0012)
        self.reg_w(0xd3, 0x008b)
        val = self.i2c_read(0x01)
        if val != 0:
            self._log('probe 3wr vga type 0a ? ret: %04x', val)
            return 0x16                     # ADCM2700
        return -1

    def probe_sensor(self):
        """Run the bridge's sensor discovery, returning the raw sensor id."""
        if self.sensor in ('SENSOR_MC501CB', 'SENSOR_GC0303'):
            return -1
        if self.sensor == 'SENSOR_PAS106':
            found = self._sif_probe()
            if found >= 0:
                return found
        found = self._vga_2wr_probe()
        if found >= 0:
            return found
        return self._vga_3wr_probe()

    # -- init (sd_init) ----------------------------------------------------
    _SENSOR_BY_ID = {
        0x02: 'SENSOR_TAS5130C',
        0x04: 'SENSOR_CS2102',
        0x08: 'SENSOR_HDCS2020',
        0x0a: 'SENSOR_PB0330',
        0x0c: 'SENSOR_ICM105A',
        0x0e: 'SENSOR_PAS202B',
        0x0f: 'SENSOR_PAS106',
        0x10: 'SENSOR_TAS5130C',
        0x11: 'SENSOR_HV7131R',
        0x12: 'SENSOR_TAS5130C',
        0x14: 'SENSOR_CS2102K',
        0x16: 'SENSOR_ADCM2700',
        0x29: 'SENSOR_GC0305',
        0x0303: 'SENSOR_GC0303',
        0x2030: 'SENSOR_PO2030',
        0x7620: 'SENSOR_OV7620',
        0x7631: 'SENSOR_OV7630C',
        0x7648: 'SENSOR_OV7620',
    }

    def init(self, force_sensor=None):
        """Probe the sensor and work out the supported video modes."""
        sensor_id = self.probe_sensor()
        self._log('probe sensor -> %04x', sensor_id & 0xffff)

        if force_sensor is not None:
            self.sensor = force_sensor
        elif sensor_id == -1:
            if self.sensor not in ('SENSOR_MC501CB', 'SENSOR_GC0303'):
                self._log('unknown sensor - set to TAS5130C')
                self.sensor = 'SENSOR_TAS5130C'
        elif sensor_id == 0:
            # HV7131: sub-type lives in sensor register 0
            sub = self.i2c_read(0x00)
            self._log('sensor hv7131 type %d', sub)
            self.sensor = 'SENSOR_HV7131B' if sub in (0, 1) else 'SENSOR_HV7131R'
        elif sensor_id in (0x13, 0x15):
            self.sensor = ('SENSOR_MT9V111_1' if self.bridge == BRIDGE_ZC301
                           else 'SENSOR_MT9V111_3')
        elif sensor_id in self._SENSOR_BY_ID:
            self.sensor = self._SENSOR_BY_ID[sensor_id]
        else:
            raise ZC3xxError('unknown sensor %04x' % sensor_id)

        self._log('sensor = %s', self.sensor)

        if sensor_id < 0x20:
            if sensor_id in (-1, 0x10, 0x12):
                self.reg_w(0x02, 0x0010)
            self.reg_r(0x0010)

        mode_kind = T.MODE_TB[self.sensor]
        self.modes = T.MODES[
            {0: 'sif_mode', 1: 'vga_mode'}.get(mode_kind, 'broken_vga_mode')]

        self.gamma = T.GAMMA_DEF[self.sensor]
        self.sharpness = 0 if self.sensor == 'SENSOR_PO2030' else 2
        if self.sensor == 'SENSOR_HV7131R':
            self.exposure = 0x927
        elif self.sensor == 'SENSOR_OV7620':
            self.exposure = 0x41

        self.reg_w(0x01, 0x0000)            # switch off the led
        return self.sensor

    # -- image controls ----------------------------------------------------
    def set_matrix(self):
        name = T.MATRIX_TB.get(self.sensor)
        if not name:
            return                          # matrix already loaded by the table
        matrix = getattr(T, name.upper())
        for i, val in enumerate(matrix):
            self.reg_w(val, 0x010a + i)

    def set_sharpness(self, val):
        self.sharpness = val
        lo, hi = T.SHARPNESS_TB[val]
        self.reg_w(lo, 0x01c6)
        self.reg_r(0x01c8)
        self.reg_r(0x01c9)
        self.reg_r(0x01ca)
        self.reg_w(hi, 0x01cb)

    def set_contrast(self, gamma=None, brightness=None, contrast=None):
        """Program the bridge's 16-point gamma curve and its gradient."""
        if gamma is not None:
            self.gamma = gamma
        if brightness is not None:
            self.brightness = brightness
        if contrast is not None:
            self.contrast = contrast

        tgamma = T.GAMMA_TB[self.gamma - 1]
        contrast = self.contrast - 128
        brightness = self.brightness - 128

        gr = [0] * 16
        adj = 0
        gp1 = gp2 = 0
        for i in range(16):
            g = (tgamma[i] + _cdiv(T.DELTA_B[i] * brightness, 256)
                 - _cdiv(T.DELTA_C[i] * contrast, 256) - _cdiv(adj, 2))
            g = min(255, max(0, g))
            self.reg_w(g, 0x0120 + i)       # gamma
            if contrast > 0:
                adj -= 1
            elif contrast < 0:
                adj += 1
            if i > 1:
                gr[i - 1] = _cdiv(g - gp2, 2) & 0xff
            elif i != 0:
                gr[0] = 0 if gp1 == 0 else (g - gp1) & 0xff
            gp2 = gp1
            gp1 = g
        gr[15] = _cdiv(0xff - gp2, 2) & 0xff
        for i in range(16):
            self.reg_w(gr[i], 0x0130 + i)   # gradient

    def set_exposure(self, val):
        self.exposure = val
        if self.sensor == 'SENSOR_HV7131R':
            self.i2c_write(0x25, (val >> 9) & 0xff, 0x00)
            self.i2c_write(0x26, (val >> 1) & 0xff, 0x00)
            self.i2c_write(0x27, (val << 7) & 0xff, 0x00)
        elif self.sensor == 'SENSOR_OV7620':
            self.i2c_write(0x10, val & 0xff, 0x00)

    def get_exposure(self):
        if self.sensor == 'SENSOR_HV7131R':
            return ((self.i2c_read(0x25) << 9) | (self.i2c_read(0x26) << 1)
                    | (self.i2c_read(0x27) >> 7))
        if self.sensor == 'SENSOR_OV7620':
            return self.i2c_read(0x10)
        return -1

    def set_autogain(self, val):
        self.autogain = val
        if self.sensor == 'SENSOR_OV7620':
            self.i2c_write(0x13, 0xa3 if val else 0x80, 0x00)
        else:
            self.reg_w(0x42 if val else 0x02, 0x0180)

    @property
    def quality(self):
        return T.JPEG_QUAL[self.reg08 >> 1]

    def _select_quality(self, quality):
        """Map a requested quality onto one of the bridge's three steps."""
        index = len(T.JPEG_QUAL) - 1
        for index, qual in enumerate(T.JPEG_QUAL):        # noqa: B007
            if quality <= qual:
                break
        self.reg08 = (index << 1) | 1

    def set_quality(self, quality=None):
        """Pick one of the bridge's JPEG quality steps (50, 75 or 87)."""
        if quality is not None:
            self._select_quality(quality)
        if self.jpeg_hdr:
            self.jpeg_hdr = jpeg_header(self.width, self.height, self.quality)
            if self._assembler is not None:
                self._assembler.jpeg_hdr = self.jpeg_hdr
        self.reg_w(self.reg08, 0x0008)

    def set_light_frequency(self, val):
        """0 = no flicker filter, 1 = 50Hz, 2 = 60Hz."""
        self.light_frequency = val
        names = T.FREQ_TB.get(self.sensor)
        if not names:
            return
        scale = self.modes[self.mode][2]    # .priv: 1 for the half-size mode
        name = names[val * 2 + (1 if scale else 0)]
        if name == 'NULL':
            return
        self.usb_exchange(name)
        if self.sensor == 'SENSOR_GC0305':
            if scale and val == 1:
                self.reg_w(0x85, 0x018d)
        elif self.sensor == 'SENSOR_OV7620':
            if not scale:
                self.reg_w(0x40 if val else 0x44, 0x0002)
        elif self.sensor == 'SENSOR_PAS202B':
            self.reg_w(0x00, 0x01a7)

    # -- start / stop (sd_start) -------------------------------------------
    def start(self, mode=0, alt=None, quality=None,
              transfers=8, packets_per_transfer=32, on_frame=None):
        """Configure the sensor for `mode` and begin isochronous streaming.

        `mode` indexes `self.modes` (0 = half size, 1 = full size).
        """
        if self.streaming:
            raise ZC3xxError('already streaming')
        if not self.modes:
            raise ZC3xxError('call init() first')

        self.mode = mode
        self.width, self.height, priv = self.modes[mode]
        if quality is not None:
            self._select_quality(quality)
        self.jpeg_hdr = jpeg_header(self.width, self.height, self.quality)

        # pick the fattest alternate setting we can; on a 12Mbps bus even the
        # largest (896 bytes/frame) is barely enough for 640x480 JPEG.
        if alt is None:
            alt = self._max_alt()
        self._alt = alt
        self.handle.setInterfaceAltSetting(IFACE, alt)

        if self.sensor == 'SENSOR_HV7131R':
            self.probe_sensor()
        elif self.sensor == 'SENSOR_PAS106':
            self.usb_exchange('pas106b_Initial_com')

        init_full, init_scale = T.INIT_TB[self.sensor]
        self.usb_exchange(init_scale if priv else init_full)

        if self.sensor in ('SENSOR_ADCM2700', 'SENSOR_GC0305', 'SENSOR_OV7620',
                           'SENSOR_PO2030', 'SENSOR_TAS5130C', 'SENSOR_GC0303'):
            self.reg_r(0x0002)
            self.reg_w(0x09, 0x01ad)        # (from windows traces)
            self.reg_w(0x15, 0x01ae)
            if self.sensor != 'SENSOR_TAS5130C':
                self.reg_w(0x0d, 0x003a)
                self.reg_w(0x02, 0x003b)
                self.reg_w(0x00, 0x0038)
        elif self.sensor in ('SENSOR_HV7131R', 'SENSOR_PAS202B'):
            self.reg_w(0x03, 0x003b)
            self.reg_w(0x0c, 0x003a)
            self.reg_w(0x0b, 0x0039)
            if self.sensor == 'SENSOR_HV7131R':
                self.reg_w(0x50, 0x011d)    # global gain

        self.set_matrix()
        if self.sensor in ('SENSOR_ADCM2700', 'SENSOR_OV7620'):
            self.reg_r(0x0008)
            self.reg_w(0x00, 0x0008)
        elif self.sensor in ('SENSOR_PAS202B', 'SENSOR_GC0305',
                             'SENSOR_HV7131R', 'SENSOR_TAS5130C'):
            self.reg_r(0x0008)
            self.reg_w(0x03, 0x0008)
        elif self.sensor == 'SENSOR_PO2030':
            self.reg_w(0x03, 0x0008)

        self.set_sharpness(self.sharpness)

        if self.sensor not in ('SENSOR_CS2102K', 'SENSOR_HDCS2020',
                               'SENSOR_OV7630C'):
            self.set_contrast()             # gamma set in the init table otherwise

        self.set_matrix()                   # the driver does this twice

        if self.sensor in ('SENSOR_OV7620', 'SENSOR_PAS202B'):
            self.reg_r(0x0180)
            self.reg_w(0x00, 0x0180)

        self.set_quality()
        # start with bit-rate control disabled; the BRC loop enables it if the
        # bridge's FIFO starts overflowing
        self.reg_w(0x00, 0x0007)
        self._reg07 = 0
        self._brc_good = 0
        self._brc_next = time.time() + 0.1

        if self.light_frequency:
            self.set_light_frequency(self.light_frequency)

        if self.sensor == 'SENSOR_ADCM2700':
            self.reg_w(0x09, 0x01ad)
            self.reg_w(0x15, 0x01ae)
            self.reg_w(0x02, 0x0180)
            self.reg_w(0x40, 0x0117)
        elif self.sensor == 'SENSOR_HV7131R':
            self.set_exposure(self.exposure)
            self.reg_w(0x00, 0x01a7)        # calc global mean
        elif self.sensor in ('SENSOR_GC0305', 'SENSOR_TAS5130C',
                             'SENSOR_PAS202B', 'SENSOR_PO2030'):
            if self.sensor in ('SENSOR_GC0305', 'SENSOR_TAS5130C'):
                self.reg_w(0x09, 0x01ad)
                self.reg_w(0x15, 0x01ae)
            self.reg_r(0x0180)
        elif self.sensor == 'SENSOR_OV7620':
            self.reg_w(0x09, 0x01ad)
            self.reg_w(0x15, 0x01ae)
            self.i2c_read(0x13)
            self.i2c_write(0x13, 0xa3, 0x00)
            self.reg_w(0x40, 0x0117)
            self.reg_r(0x0180)

        self.set_autogain(self.autogain)

        self._assembler = FrameAssembler(self.jpeg_hdr)
        self._on_frame = on_frame
        # set the flag first: the completion callback resubmits only while
        # streaming, and a transfer can complete the moment it is submitted
        self.streaming = True
        try:
            self._submit_transfers(transfers, packets_per_transfer)
        except Exception:
            self.streaming = False
            raise
        return self.width, self.height

    @property
    def frames_dropped(self):
        """Frames abandoned because a new one started before the old ended."""
        return self._assembler.frames_dropped if self._assembler else 0

    @property
    def frame_header(self):
        """The bridge's own header from the last frame it started."""
        return self._assembler.last_header if self._assembler else None

    def _max_alt(self):
        """Largest alternate setting of the video interface."""
        device = self.handle.getDevice()
        best = 0
        best_size = -1
        for settings in device.iterSettings():
            if settings.getNumber() != IFACE:
                continue
            for endpoint in settings:
                if endpoint.getAddress() != EP_VIDEO:
                    continue
                size = endpoint.getMaxPacketSize()
                if size > best_size:
                    best_size = size
                    best = settings.getAlternateSetting()
        if best_size <= 0:
            raise ZC3xxError('no isochronous endpoint 0x%02x found' % EP_VIDEO)
        self._log('alt %d, %d bytes/packet', best, best_size)
        return best

    def _packet_size(self, alt):
        device = self.handle.getDevice()
        for settings in device.iterSettings():
            if settings.getNumber() == IFACE and \
                    settings.getAlternateSetting() == alt:
                for endpoint in settings:
                    if endpoint.getAddress() == EP_VIDEO:
                        return endpoint.getMaxPacketSize()
        raise ZC3xxError('alt setting %d has no endpoint 0x%02x' % (alt, EP_VIDEO))

    def _submit_transfers(self, count, packets):
        size = self._packet_size(self._alt)
        self._transfers = []
        for _ in range(count):
            transfer = self.handle.getTransfer(iso_packets=packets)
            transfer.setIsochronous(EP_VIDEO, packets * size,
                                    callback=self._iso_callback)
            transfer.submit()
            self._transfers.append(transfer)

    def _iso_callback(self, transfer):
        if transfer.getStatus() == usb1.TRANSFER_COMPLETED:
            for status, data in transfer.iterISO():
                if status != usb1.TRANSFER_COMPLETED or not data:
                    continue
                frame = self._assembler.feed(data)
                if frame is not None and self._on_frame is not None:
                    self._on_frame(frame)
        if self.streaming:
            with contextlib.suppress(usb1.USBError):
                transfer.submit()

    # -- bit rate control (transfer_update) --------------------------------
    def poll(self):
        """Run the bridge's bit-rate-control loop; call this periodically.

        Bit 0 of register 0x11 flags a FIFO overflow.  On overflow we tighten
        the allowed bytes per isochronous packet, and back off again after ten
        clean polls, exactly like the kernel driver's work queue.
        """
        now = time.time()
        if not self.streaming or now < self._brc_next:
            return
        self._brc_next = now + 0.1

        change = self.reg_r(0x0011) & 0x01
        if change:                          # overflow
            self._brc_good = 0
            if self._reg07 == 0:
                self._reg07 = 0x32          # allow 98 bytes/unit
            elif self._reg07 > 2:
                self._reg07 -= 2
            else:
                change = 0
        else:
            self._brc_good += 1
            if self._brc_good >= 10:
                self._brc_good = 0
                if self._reg07:
                    change = 1
                    if self._reg07 < 0x32:
                        self._reg07 += 2
                    else:
                        self._reg07 = 0
        if change:
            self.reg_w(self._reg07, 0x0007)

    def stop(self):
        if not self.streaming:
            return
        self.streaming = False
        for transfer in self._transfers:
            with contextlib.suppress(usb1.USBError):
                transfer.cancel()
        deadline = time.time() + 1.0
        while time.time() < deadline and \
                any(t.isSubmitted() for t in self._transfers):
            if self.context is not None:
                self.context.handleEventsTimeout(0.05)
            else:
                time.sleep(0.01)
        self._transfers = []
        with contextlib.suppress(usb1.USBError):
            self.handle.setInterfaceAltSetting(IFACE, 0)
        with contextlib.suppress(usb1.USBError):
            self._send_unknown(self.sensor)
        with contextlib.suppress(usb1.USBError):
            self.reg_w(0x01, 0x0000)        # led off


def _cdiv(numerator, denominator):
    """Integer division that truncates toward zero, the way C does."""
    quotient = abs(numerator) // abs(denominator)
    if (numerator < 0) != (denominator < 0):
        return -quotient
    return quotient


# ---------------------------------------------------------------------------
# device discovery
# ---------------------------------------------------------------------------
def find_devices(context):
    """Yield every attached device this driver recognises."""
    for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == VENDOR_ID and \
                device.getProductID() in PRODUCT_IDS:
            yield device


@contextlib.contextmanager
def open_camera(context=None, debug=False, force_sensor=None):
    """Context manager yielding an initialised :class:`Camera`."""
    own_context = context is None
    if own_context:
        context = usb1.USBContext()
        context.open()
    try:
        device = next(iter(find_devices(context)), None)
        if device is None:
            raise ZC3xxError(
                'no ZC3xx camera found (looked for %04x:%s)'
                % (VENDOR_ID, '/'.join('%04x' % p for p in PRODUCT_IDS)))
        handle = device.open()
        try:
            try:
                handle.claimInterface(IFACE)
            except usb1.USBErrorAccess:
                raise ZC3xxError(
                    'the camera is busy: another process (or a browser tab '
                    'using WebUSB) still holds interface %d. Close it, or '
                    'unplug and replug the camera.' % IFACE)
            camera = Camera(handle, device.getProductID(),
                            debug=debug, context=context)
            camera.init(force_sensor=force_sensor)
            try:
                yield camera
            finally:
                camera.stop()
        finally:
            with contextlib.suppress(usb1.USBError):
                handle.releaseInterface(IFACE)
            handle.close()
    finally:
        if own_context:
            context.close()
