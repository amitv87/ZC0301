// Userspace driver for the Z-Star/Vimicro ZC0301(P) bridge, over WebUSB.
//
// A port of the Linux gspca `zc3xx` driver.  The register/init tables in
// zc3xx-tables.js are generated from the kernel sources; this file is the
// control flow around them: sensor probing, the start-up sequence, the image
// controls and the isochronous frame assembler.
//
// The bridge sends JPEG data with the tables stripped out, so we synthesise a
// JFIF header (jpegHeader) and glue it in front of every frame.

import * as T from './zc3xx-tables.js';

export const VENDOR_ID = 0x0ac8;
export const PRODUCT_IDS = [0x0301, 0x0302, 0x301b, 0x303b, 0x305b, 0x307b];

const IFACE = 0;
const EP_VIDEO = 1;           // endpoint 0x81, isochronous IN
const REG08_DEF = 3;          // bits 1-2 select JPEG quality; 3 -> 75%
const BRIDGE_ZC301 = 0;
const BRIDGE_ZC303 = 1;

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

export class ZC3xxError extends Error {
  constructor(message) {
    super(message);
    this.name = 'ZC3xxError';
  }
}

/**
 * Chromium's Windows USB backend is a stub: IsochronousTransferIn in
 * services/device/usb/usb_device_handle_win.cc reports TRANSFER_ERROR without
 * attempting anything, which blink surfaces as a NetworkError reading
 * "A transfer error has occurred."  Nothing on this side can work around it,
 * so say so rather than leaving the caller staring at a generic failure.
 */
function explainIsoFailure(error) {
  const onWindows = /Windows/i.test(
    (navigator.userAgentData && navigator.userAgentData.platform) ||
    navigator.userAgent || '');
  if (error && error.name === 'NetworkError' && onWindows) {
    return new ZC3xxError(
      'Isochronous transfers are not implemented in Chromium on Windows, so ' +
      'WebUSB cannot stream video from this camera there (the bridge offers ' +
      'no bulk endpoint to fall back to). The libusb driver does work on ' +
      'Windows: run "python zc3xx_capture.py stream" and open the MJPEG ' +
      'viewer it prints. macOS, Linux and ChromeOS stream over WebUSB fine.');
  }
  if (error && error.name === 'NetworkError') {
    return new ZC3xxError(
      'The isochronous transfer failed. The camera is full speed, so alt ' +
      'setting 7 asks for 896 of the 1023 bytes per frame the bus has; if ' +
      'something else on the same controller already reserved bandwidth, try ' +
      'a lower alt setting or a different port. (' + error.message + ')');
  }
  return error;
}

// ---------------------------------------------------------------------------
// generated-table helpers
// ---------------------------------------------------------------------------
const actionCache = new Map();

/** Unpack one "a0110002..." table into [request, value, index] triples. */
function unpackActions(name) {
  if (actionCache.has(name)) return actionCache.get(name);
  const hex = T.ACTIONS[name];
  if (hex === undefined) throw new ZC3xxError(`unknown table ${name}`);
  const out = [];
  for (let i = 0; i < hex.length; i += 8) {
    out.push([
      parseInt(hex.slice(i, i + 2), 16),
      parseInt(hex.slice(i + 2, i + 4), 16),
      parseInt(hex.slice(i + 4, i + 8), 16),
    ]);
  }
  actionCache.set(name, out);
  return out;
}

// ---------------------------------------------------------------------------
// JPEG header synthesis (port of gspca's jpeg.h)
// ---------------------------------------------------------------------------
export function jpegHeader(width, height, quality = 75, samplesY = 0x21) {
  const hdr = T.JPEG_HEAD.slice();
  const off = T.JPEG_HEIGHT_OFFSET;
  hdr[off + 0] = (height >> 8) & 0xff;
  hdr[off + 1] = height & 0xff;
  hdr[off + 2] = (width >> 8) & 0xff;
  hdr[off + 3] = width & 0xff;
  hdr[off + 6] = samplesY;

  let scale;
  if (quality <= 0) scale = 5000;
  else if (quality < 50) scale = Math.floor(5000 / quality);
  else scale = 200 - quality * 2;

  for (let i = 0; i < 64; i++) {
    for (const base of [T.JPEG_QT0_OFFSET, T.JPEG_QT1_OFFSET]) {
      const value = Math.floor((T.JPEG_HEAD[base + i] * scale + 50) / 100);
      hdr[base + i] = Math.min(255, Math.max(1, value));
    }
  }
  return hdr;
}

// ---------------------------------------------------------------------------
// frame assembly (port of zc3xx's sd_pkt_scan)
// ---------------------------------------------------------------------------
//
// Every frame starts with
//   ff d8 ff fe 00 0e 00 00 ss ss 00 01 ww ww hh hh pp pp
// (SOI plus an 18-byte COM segment carrying the frame sequence number, the
// window dimensions and the packet sequence number) and ends with ff d9 plus
// one trailing byte the kernel driver discards too.
export class FrameAssembler {
  constructor(jpegHdr) {
    this.jpegHdr = jpegHdr;
    this.chunks = [];
    this.length = 0;
    this.inFrame = false;
    this.framesStarted = 0;
    this.framesDropped = 0;
    this.lastHeader = null;
  }

  /** Consume one isochronous packet; returns a complete frame or null. */
  feed(data) {
    if (!data.length) return null;

    // end of frame?
    if (data.length >= 3 && data[data.length - 3] === 0xff &&
        data[data.length - 2] === 0xd9) {
      if (!this.inFrame) return null;
      this.push(data.subarray(0, data.length - 1));
      const frame = this.flatten();
      this.reset();
      return frame;
    }

    // start of frame?
    if (data.length >= 2 && data[0] === 0xff && data[1] === 0xd8) {
      if (this.inFrame) this.framesDropped++;
      this.lastHeader = FrameAssembler.parseHeader(data);
      this.reset();
      this.push(this.jpegHdr);
      this.inFrame = true;
      this.framesStarted++;
      data = data.subarray(18);
    }

    if (this.inFrame) this.push(data);
    return null;
  }

  push(chunk) {
    this.chunks.push(chunk);
    this.length += chunk.length;
  }

  reset() {
    this.chunks = [];
    this.length = 0;
    this.inFrame = false;
  }

  flatten() {
    const out = new Uint8Array(this.length);
    let offset = 0;
    for (const chunk of this.chunks) {
      out.set(chunk, offset);
      offset += chunk.length;
    }
    return out;
  }

  static parseHeader(data) {
    if (data.length < 18) return null;
    return {
      sequence: (data[8] << 8) | data[9],
      width: (data[12] << 8) | data[13],
      height: (data[14] << 8) | data[15],
      packet: (data[16] << 8) | data[17],
    };
  }
}

// ---------------------------------------------------------------------------
// the camera
// ---------------------------------------------------------------------------
export class Camera {
  constructor(device, { debug = false, onLog = null } = {}) {
    this.device = device;
    this.debug = debug;
    this.onLog = onLog;

    this.bridge = device.productId === 0x301b ? BRIDGE_ZC301 : BRIDGE_ZC303;
    // gspca seeds sd->sensor from driver_info, which is 0 (== ADCM2700) for
    // the auto-probed models.
    this.sensor = T.SENSORS[0];
    this.chipRevision = 0;
    this.reg08 = REG08_DEF;

    this.modes = [];
    this.mode = 0;
    this.width = 0;
    this.height = 0;
    this.jpegHdr = null;

    // image controls, defaults from sd_init_controls()
    this.brightness = 128;
    this.contrast = 128;
    this.gamma = 4;
    this.sharpness = 2;
    this.autogain = 1;
    this.exposure = null;
    this.lightFrequency = 0;      // 0 = off, 1 = 50Hz, 2 = 60Hz

    this.streaming = false;
    this.alt = 0;
    this.assembler = null;
    this.onFrame = null;
    this.onError = null;
    this.isoDone = null;
    this.brcDone = null;
    this.reg07 = 0;
    this.brcGood = 0;
  }

  log(message) {
    if (this.debug) console.log('[zc3xx]', message);
    if (this.onLog) this.onLog(message);
  }

  // -- plumbing ------------------------------------------------------------
  async regW(value, index) {
    const result = await this.device.controlTransferOut({
      requestType: 'vendor', recipient: 'device',
      request: 0xa0, value, index,
    });
    if (result.status !== 'ok') {
      throw new ZC3xxError(`reg_w(0x${index.toString(16)}) ${result.status}`);
    }
  }

  async regR(index) {
    const result = await this.device.controlTransferIn({
      requestType: 'vendor', recipient: 'device',
      request: 0xa1, value: 0x01, index,
    }, 1);
    if (result.status !== 'ok' || !result.data || result.data.byteLength < 1) {
      throw new ZC3xxError(`reg_r(0x${index.toString(16)}) ${result.status}`);
    }
    return result.data.getUint8(0);
  }

  async i2cWrite(reg, valLo, valHi) {
    await this.regW(reg, 0x0092);
    await this.regW(valLo, 0x0093);
    await this.regW(valHi, 0x0094);
    await this.regW(0x01, 0x0090);          // write command
    await sleep(1);
    const status = await this.regR(0x0091);
    if (status !== 0) this.log(`i2c_w status error ${status.toString(16)}`);
    return status;
  }

  async i2cRead(reg) {
    await this.regW(reg, 0x0092);
    await this.regW(0x02, 0x0090);          // read command
    await sleep(20);
    const status = await this.regR(0x0091);
    if (status !== 0) this.log(`i2c_r status error ${status.toString(16)}`);
    const lo = await this.regR(0x0095);
    const hi = await this.regR(0x0096);
    return lo | (hi << 8);
  }

  /** Replay one of the driver's usb_action tables. */
  async usbExchange(table) {
    const actions = typeof table === 'string' ? unpackActions(table) : table;
    for (const [request, value, index] of actions) {
      if (request === 0xa0) await this.regW(value, index);
      else if (request === 0xa1) await this.regR(index);
      else if (request === 0xaa) await this.i2cWrite(value, index & 0xff, index >> 8);
      else if (request === 0xbb) await this.i2cWrite(index >> 8, index & 0xff, value);
      else await sleep(index);               // 0xdd: delay
      await sleep(1);
    }
  }

  // -- sensor probing ------------------------------------------------------
  async start2wrProbe(sensor) {
    await this.regW(0x01, 0x0000);
    await this.regW(sensor, 0x0010);
    await this.regW(0x01, 0x0001);
    await this.regW(0x03, 0x0012);
    await this.regW(0x01, 0x0012);
  }

  async sendUnknown(sensor) {
    await this.regW(0x01, 0x0000);          // bridge reset
    if (sensor === 'SENSOR_PAS106') {
      await this.regW(0x03, 0x003a);
      await this.regW(0x0c, 0x003b);
      await this.regW(0x08, 0x0038);
    } else if (['SENSOR_ADCM2700', 'SENSOR_GC0305', 'SENSOR_OV7620',
                'SENSOR_MT9V111_1', 'SENSOR_MT9V111_3', 'SENSOR_PB0330',
                'SENSOR_PO2030'].includes(sensor)) {
      await this.regW(0x0d, 0x003a);
      await this.regW(0x02, 0x003b);
      await this.regW(0x00, 0x0038);
    } else if (sensor === 'SENSOR_HV7131R' || sensor === 'SENSOR_PAS202B') {
      await this.regW(0x03, 0x003b);
      await this.regW(0x0c, 0x003a);
      await this.regW(0x0b, 0x0039);
      if (sensor === 'SENSOR_PAS202B') await this.regW(0x0b, 0x0038);
    }
  }

  async sifProbe() {
    await this.start2wrProbe(0x0f);         // PAS106
    await this.regW(0x08, 0x008d);
    await sleep(150);
    const check = (((await this.i2cRead(0x00)) & 0x0f) << 4)
                | (((await this.i2cRead(0x01)) & 0xf0) >> 4);
    this.log(`probe sif 0x${check.toString(16)}`);
    if (check === 0x0007) {
      await this.sendUnknown('SENSOR_PAS106');
      return 0x0f;
    }
    return -1;
  }

  async vga2wrProbe() {
    await this.start2wrProbe(0x00);         // HV7131B
    await this.i2cWrite(0x01, 0xaa, 0x00);
    if (await this.i2cRead(0x01)) return 0x00;

    await this.start2wrProbe(0x04);         // CS2102
    await this.i2cWrite(0x01, 0xaa, 0x00);
    if (await this.i2cRead(0x01)) return 0x04;

    await this.start2wrProbe(0x06);         // OmniVision
    await this.regW(0x08, 0x008d);
    await this.i2cWrite(0x11, 0xaa, 0x00);
    const omnivision = (await this.i2cRead(0x11)) !== 0;

    if (!omnivision) {
      await this.start2wrProbe(0x08);       // HDCS2020
      await this.i2cWrite(0x1c, 0x00, 0x00);
      await this.i2cWrite(0x15, 0xaa, 0x00);
      if (await this.i2cRead(0x15)) return 0x08;

      await this.start2wrProbe(0x0a);       // PB0330
      await this.i2cWrite(0x07, 0xaa, 0xaa);
      if (await this.i2cRead(0x07)) return 0x0a;
      if (await this.i2cRead(0x03)) return 0x0a;
      if (await this.i2cRead(0x04)) return 0x0a;

      await this.start2wrProbe(0x0c);       // ICM105A
      await this.i2cWrite(0x01, 0x11, 0x00);
      if (await this.i2cRead(0x01)) return 0x0c;

      await this.start2wrProbe(0x0e);       // PAS202BCB
      await this.regW(0x08, 0x008d);
      await this.i2cWrite(0x03, 0xaa, 0x00);
      await sleep(50);
      if (await this.i2cRead(0x03)) {
        await this.sendUnknown('SENSOR_PAS202B');
        return 0x0e;
      }

      await this.start2wrProbe(0x02);       // TAS5130C
      await this.i2cWrite(0x01, 0xaa, 0x00);
      if (await this.i2cRead(0x01)) return 0x02;
    }

    await this.regR(0x0010);
    await this.regR(0x0010);
    await this.regW(0x01, 0x0000);
    await this.regW(0x01, 0x0001);
    await this.regW(0x06, 0x0010);          // OmniVision
    await this.regW(0xa1, 0x008b);
    await this.regW(0x08, 0x008d);
    await sleep(500);
    await this.regW(0x01, 0x0012);
    await this.i2cWrite(0x12, 0x80, 0x00);  // sensor reset
    const value = ((await this.i2cRead(0x0a)) << 8) | (await this.i2cRead(0x0b));
    this.log(`probe 2wr ov vga 0x${value.toString(16)}`);
    if (value === 0x7631) {
      await this.regW(0x06, 0x0010);        // OV7630C
    } else if (value !== 0x7620 && value !== 0x7648) {
      return -1;
    }
    return value;
  }

  async vga3wrProbe() {
    await this.regW(0x02, 0x0010);
    await this.regR(0x0010);
    await this.regW(0x01, 0x0000);
    await this.regW(0x00, 0x0010);
    await this.regW(0x01, 0x0001);
    await this.regW(0x91, 0x008b);
    await this.regW(0x03, 0x0012);
    await this.regW(0x01, 0x0012);
    await this.regW(0x05, 0x0012);
    for (const reg of [0x14, 0x15, 0x16]) {
      if (await this.i2cRead(reg)) return 0x11;   // HV7131R
    }

    await this.regW(0x02, 0x0010);
    let value = ((await this.regR(0x000b)) << 8) | (await this.regR(0x000a));
    this.log(`probe 3wr vga 1 0x${value.toString(16)}`);
    await this.regR(0x0010);
    if ((value & 0xff00) === 0x6400) return 0x02;  // TAS5130C
    for (const [revision, internalId] of T.CHIP_REVISION_SENSOR) {
      if (revision === value) {
        this.chipRevision = value;
        await this.sendUnknown('SENSOR_PB0330');
        return internalId;
      }
    }

    await this.regW(0x01, 0x0000);          // check PB0330
    await this.regW(0x01, 0x0001);
    await this.regW(0xdd, 0x008b);
    await this.regW(0x0a, 0x0010);
    await this.regW(0x03, 0x0012);
    await this.regW(0x01, 0x0012);
    if (await this.i2cRead(0x00)) {
      this.log('probe 3wr vga type 0a');
      return 0x0a;
    }

    await this.regW(0x01, 0x0000);          // check GC0303 / GC0305
    await this.regW(0x01, 0x0001);
    await this.regW(0x98, 0x008b);
    await this.regW(0x01, 0x0010);
    await this.regW(0x03, 0x0012);
    await sleep(2);
    await this.regW(0x01, 0x0012);
    value = await this.i2cRead(0x00);
    if (value) {
      this.log(`probe 3wr vga type ${value.toString(16)}`);
      if (value === 0x0011) return 0x0303;  // GC0303
      if (value === 0x0029) await this.sendUnknown('SENSOR_GC0305');
      return value;
    }

    await this.regW(0x01, 0x0000);          // check OmniVision
    await this.regW(0x01, 0x0001);
    await this.regW(0xa1, 0x008b);
    await this.regW(0x08, 0x008d);
    await this.regW(0x06, 0x0010);
    await this.regW(0x01, 0x0012);
    await this.regW(0x05, 0x0012);
    if ((await this.i2cRead(0x1c)) === 0x007f &&
        (await this.i2cRead(0x1d)) === 0x00a2) {
      await this.sendUnknown('SENSOR_OV7620');
      return 0x06;
    }

    await this.regW(0x01, 0x0000);
    await this.regW(0x00, 0x0002);
    await this.regW(0x01, 0x0010);
    await this.regW(0x01, 0x0001);
    await this.regW(0xee, 0x008b);
    await this.regW(0x03, 0x0012);
    await this.regW(0x01, 0x0012);
    await this.regW(0x05, 0x0012);
    value = ((await this.i2cRead(0x00)) << 8) | (await this.i2cRead(0x01));
    this.log(`probe 3wr vga 2 0x${value.toString(16)}`);
    if (value === 0x2030) {
      this.log(`sensor PO2030 rev 0x${(await this.i2cRead(0x02)).toString(16)}`);
      await this.sendUnknown('SENSOR_PO2030');
      return value;
    }

    await this.regW(0x01, 0x0000);
    await this.regW(0x0a, 0x0010);
    await this.regW(0xd3, 0x008b);
    await this.regW(0x01, 0x0001);
    await this.regW(0x03, 0x0012);
    await this.regW(0x01, 0x0012);
    await this.regW(0x05, 0x0012);
    await this.regW(0xd3, 0x008b);
    if (await this.i2cRead(0x01)) {
      this.log('probe 3wr vga type 0a ? -> adcm2700');
      return 0x16;                          // ADCM2700
    }
    return -1;
  }

  async probeSensor() {
    if (this.sensor === 'SENSOR_MC501CB' || this.sensor === 'SENSOR_GC0303') {
      return -1;
    }
    if (this.sensor === 'SENSOR_PAS106') {
      const found = await this.sifProbe();
      if (found >= 0) return found;
    }
    const found = await this.vga2wrProbe();
    if (found >= 0) return found;
    return this.vga3wrProbe();
  }

  // -- init (sd_init) ------------------------------------------------------
  static SENSOR_BY_ID = {
    0x02: 'SENSOR_TAS5130C', 0x04: 'SENSOR_CS2102', 0x08: 'SENSOR_HDCS2020',
    0x0a: 'SENSOR_PB0330', 0x0c: 'SENSOR_ICM105A', 0x0e: 'SENSOR_PAS202B',
    0x0f: 'SENSOR_PAS106', 0x10: 'SENSOR_TAS5130C', 0x11: 'SENSOR_HV7131R',
    0x12: 'SENSOR_TAS5130C', 0x14: 'SENSOR_CS2102K', 0x16: 'SENSOR_ADCM2700',
    0x29: 'SENSOR_GC0305', 0x0303: 'SENSOR_GC0303', 0x2030: 'SENSOR_PO2030',
    0x7620: 'SENSOR_OV7620', 0x7631: 'SENSOR_OV7630C', 0x7648: 'SENSOR_OV7620',
  };

  async init(forceSensor = null) {
    const sensorId = await this.probeSensor();
    this.log(`probe sensor -> 0x${(sensorId & 0xffff).toString(16)}`);

    if (forceSensor) {
      this.sensor = forceSensor;
    } else if (sensorId === -1) {
      this.log('unknown sensor - set to TAS5130C');
      this.sensor = 'SENSOR_TAS5130C';
    } else if (sensorId === 0) {
      const sub = await this.i2cRead(0x00);   // HV7131 sub-type
      this.log(`sensor hv7131 type ${sub}`);
      this.sensor = (sub === 0 || sub === 1) ? 'SENSOR_HV7131B' : 'SENSOR_HV7131R';
    } else if (sensorId === 0x13 || sensorId === 0x15) {
      this.sensor = this.bridge === BRIDGE_ZC301
        ? 'SENSOR_MT9V111_1' : 'SENSOR_MT9V111_3';
    } else if (Camera.SENSOR_BY_ID[sensorId]) {
      this.sensor = Camera.SENSOR_BY_ID[sensorId];
    } else {
      throw new ZC3xxError(`unknown sensor 0x${sensorId.toString(16)}`);
    }
    this.log(`sensor = ${this.sensor}`);

    if (sensorId < 0x20) {
      if (sensorId === -1 || sensorId === 0x10 || sensorId === 0x12) {
        await this.regW(0x02, 0x0010);
      }
      await this.regR(0x0010);
    }

    const kind = T.MODE_TB[this.sensor];
    this.modes = T.MODES[{ 0: 'sif_mode', 1: 'vga_mode' }[kind] || 'broken_vga_mode'];

    this.gamma = T.GAMMA_DEF[this.sensor];
    this.sharpness = this.sensor === 'SENSOR_PO2030' ? 0 : 2;
    if (this.sensor === 'SENSOR_HV7131R') this.exposure = 0x927;
    else if (this.sensor === 'SENSOR_OV7620') this.exposure = 0x41;

    await this.regW(0x01, 0x0000);          // switch off the led
    return this.sensor;
  }

  // -- image controls ------------------------------------------------------
  async setMatrix() {
    const name = T.MATRIX_TB[this.sensor];
    if (!name) return;                      // matrix already loaded by the table
    const matrix = T[name.toUpperCase()];
    for (let i = 0; i < matrix.length; i++) await this.regW(matrix[i], 0x010a + i);
  }

  async setSharpness(value) {
    this.sharpness = value;
    const [lo, hi] = T.SHARPNESS_TB[value];
    await this.regW(lo, 0x01c6);
    await this.regR(0x01c8);
    await this.regR(0x01c9);
    await this.regR(0x01ca);
    await this.regW(hi, 0x01cb);
  }

  /** Program the bridge's 16-point gamma curve and its gradient. */
  async setContrast() {
    const tgamma = T.GAMMA_TB[this.gamma - 1];
    const contrast = this.contrast - 128;
    const brightness = this.brightness - 128;
    const gr = new Array(16).fill(0);
    let adj = 0, gp1 = 0, gp2 = 0;

    for (let i = 0; i < 16; i++) {
      let g = tgamma[i] + Math.trunc(T.DELTA_B[i] * brightness / 256)
            - Math.trunc(T.DELTA_C[i] * contrast / 256) - Math.trunc(adj / 2);
      g = Math.min(255, Math.max(0, g));
      await this.regW(g, 0x0120 + i);       // gamma
      if (contrast > 0) adj--;
      else if (contrast < 0) adj++;
      if (i > 1) gr[i - 1] = Math.trunc((g - gp2) / 2) & 0xff;
      else if (i !== 0) gr[0] = gp1 === 0 ? 0 : (g - gp1) & 0xff;
      gp2 = gp1;
      gp1 = g;
    }
    gr[15] = Math.trunc((0xff - gp2) / 2) & 0xff;
    for (let i = 0; i < 16; i++) await this.regW(gr[i], 0x0130 + i);  // gradient
  }

  async setExposure(value) {
    this.exposure = value;
    if (this.sensor === 'SENSOR_HV7131R') {
      await this.i2cWrite(0x25, (value >> 9) & 0xff, 0x00);
      await this.i2cWrite(0x26, (value >> 1) & 0xff, 0x00);
      await this.i2cWrite(0x27, (value << 7) & 0xff, 0x00);
    } else if (this.sensor === 'SENSOR_OV7620') {
      await this.i2cWrite(0x10, value & 0xff, 0x00);
    }
  }

  async setAutogain(value) {
    this.autogain = value;
    if (this.sensor === 'SENSOR_OV7620') {
      await this.i2cWrite(0x13, value ? 0xa3 : 0x80, 0x00);
    } else {
      await this.regW(value ? 0x42 : 0x02, 0x0180);
    }
  }

  get quality() {
    return T.JPEG_QUAL[this.reg08 >> 1];
  }

  selectQuality(quality) {
    let index = T.JPEG_QUAL.length - 1;
    for (let i = 0; i < T.JPEG_QUAL.length; i++) {
      if (quality <= T.JPEG_QUAL[i]) { index = i; break; }
    }
    this.reg08 = (index << 1) | 1;
  }

  async setQuality(quality = null) {
    if (quality !== null) this.selectQuality(quality);
    if (this.jpegHdr) {
      this.jpegHdr = jpegHeader(this.width, this.height, this.quality);
      if (this.assembler) this.assembler.jpegHdr = this.jpegHdr;
    }
    await this.regW(this.reg08, 0x0008);
  }

  /** 0 = no flicker filter, 1 = 50Hz, 2 = 60Hz. */
  async setLightFrequency(value) {
    this.lightFrequency = value;
    const names = T.FREQ_TB[this.sensor];
    if (!names) return;
    const scale = this.modes[this.mode][2];   // .priv: 1 for the half-size mode
    const name = names[value * 2 + (scale ? 1 : 0)];
    if (!name || name === 'NULL') return;
    await this.usbExchange(name);
    if (this.sensor === 'SENSOR_GC0305') {
      if (scale && value === 1) await this.regW(0x85, 0x018d);
    } else if (this.sensor === 'SENSOR_OV7620') {
      if (!scale) await this.regW(value ? 0x40 : 0x44, 0x0002);
    } else if (this.sensor === 'SENSOR_PAS202B') {
      await this.regW(0x00, 0x01a7);
    }
  }

  // -- start / stop (sd_start) ---------------------------------------------
  /** Largest alternate setting of the video interface, and its packet size. */
  findAlt(preferred = null) {
    const iface = this.device.configuration.interfaces
      .find((i) => i.interfaceNumber === IFACE);
    if (!iface) throw new ZC3xxError(`interface ${IFACE} not found`);
    let best = null;
    for (const alternate of iface.alternates) {
      const endpoint = alternate.endpoints.find(
        (e) => e.endpointNumber === EP_VIDEO && e.direction === 'in' &&
               e.type === 'isochronous');
      if (!endpoint) continue;
      const candidate = {
        alt: alternate.alternateSetting,
        packetSize: endpoint.packetSize,
      };
      if (preferred !== null) {
        if (candidate.alt === preferred) return candidate;
      } else if (!best || candidate.packetSize > best.packetSize) {
        best = candidate;
      }
    }
    if (preferred !== null) {
      throw new ZC3xxError(`alt setting ${preferred} has no iso endpoint`);
    }
    if (!best || !best.packetSize) {
      throw new ZC3xxError('no isochronous IN endpoint found');
    }
    return best;
  }

  /**
   * Configure the sensor for `mode` and begin isochronous streaming.
   * `mode` indexes this.modes (0 = half size, 1 = full size).
   */
  async start({ mode = 0, alt = null, quality = null, onFrame = null,
                depth = 6, packets = 32 } = {}) {
    if (this.streaming) throw new ZC3xxError('already streaming');
    if (!this.modes.length) throw new ZC3xxError('call init() first');

    this.mode = mode;
    const [width, height, priv] = this.modes[mode];
    this.width = width;
    this.height = height;
    if (quality !== null) this.selectQuality(quality);
    this.jpegHdr = jpegHeader(width, height, this.quality);

    // on a 12Mbps bus even the largest alt setting (896 bytes/frame) is barely
    // enough for 640x480 JPEG, so take the biggest one on offer
    const chosen = this.findAlt(alt);
    this.alt = chosen.alt;
    this.packetSize = chosen.packetSize;
    this.log(`alt ${chosen.alt}, ${chosen.packetSize} bytes/packet`);
    await this.device.selectAlternateInterface(IFACE, chosen.alt);

    if (this.sensor === 'SENSOR_HV7131R') await this.probeSensor();
    else if (this.sensor === 'SENSOR_PAS106') await this.usbExchange('pas106b_Initial_com');

    const [initFull, initScale] = T.INIT_TB[this.sensor];
    await this.usbExchange(priv ? initScale : initFull);

    if (['SENSOR_ADCM2700', 'SENSOR_GC0305', 'SENSOR_OV7620', 'SENSOR_PO2030',
         'SENSOR_TAS5130C', 'SENSOR_GC0303'].includes(this.sensor)) {
      await this.regR(0x0002);
      await this.regW(0x09, 0x01ad);        // (from windows traces)
      await this.regW(0x15, 0x01ae);
      if (this.sensor !== 'SENSOR_TAS5130C') {
        await this.regW(0x0d, 0x003a);
        await this.regW(0x02, 0x003b);
        await this.regW(0x00, 0x0038);
      }
    } else if (this.sensor === 'SENSOR_HV7131R' || this.sensor === 'SENSOR_PAS202B') {
      await this.regW(0x03, 0x003b);
      await this.regW(0x0c, 0x003a);
      await this.regW(0x0b, 0x0039);
      if (this.sensor === 'SENSOR_HV7131R') await this.regW(0x50, 0x011d);
    }

    await this.setMatrix();
    if (this.sensor === 'SENSOR_ADCM2700' || this.sensor === 'SENSOR_OV7620') {
      await this.regR(0x0008);
      await this.regW(0x00, 0x0008);
    } else if (['SENSOR_PAS202B', 'SENSOR_GC0305', 'SENSOR_HV7131R',
                'SENSOR_TAS5130C'].includes(this.sensor)) {
      await this.regR(0x0008);
      await this.regW(0x03, 0x0008);
    } else if (this.sensor === 'SENSOR_PO2030') {
      await this.regW(0x03, 0x0008);
    }

    await this.setSharpness(this.sharpness);

    if (!['SENSOR_CS2102K', 'SENSOR_HDCS2020', 'SENSOR_OV7630C'].includes(this.sensor)) {
      await this.setContrast();             // gamma comes from the init table otherwise
    }
    await this.setMatrix();                 // the driver does this twice

    if (this.sensor === 'SENSOR_OV7620' || this.sensor === 'SENSOR_PAS202B') {
      await this.regR(0x0180);
      await this.regW(0x00, 0x0180);
    }

    await this.setQuality();
    // start with bit-rate control off; the BRC loop turns it on if the
    // bridge's FIFO starts overflowing
    await this.regW(0x00, 0x0007);
    this.reg07 = 0;
    this.brcGood = 0;

    if (this.lightFrequency) await this.setLightFrequency(this.lightFrequency);

    if (this.sensor === 'SENSOR_ADCM2700') {
      await this.regW(0x09, 0x01ad);
      await this.regW(0x15, 0x01ae);
      await this.regW(0x02, 0x0180);
      await this.regW(0x40, 0x0117);
    } else if (this.sensor === 'SENSOR_HV7131R') {
      await this.setExposure(this.exposure);
      await this.regW(0x00, 0x01a7);        // calc global mean
    } else if (['SENSOR_GC0305', 'SENSOR_TAS5130C', 'SENSOR_PAS202B',
                'SENSOR_PO2030'].includes(this.sensor)) {
      if (this.sensor === 'SENSOR_GC0305' || this.sensor === 'SENSOR_TAS5130C') {
        await this.regW(0x09, 0x01ad);
        await this.regW(0x15, 0x01ae);
      }
      await this.regR(0x0180);
    } else if (this.sensor === 'SENSOR_OV7620') {
      await this.regW(0x09, 0x01ad);
      await this.regW(0x15, 0x01ae);
      await this.i2cRead(0x13);
      await this.i2cWrite(0x13, 0xa3, 0x00);
      await this.regW(0x40, 0x0117);
      await this.regR(0x0180);
    }

    await this.setAutogain(this.autogain);

    this.assembler = new FrameAssembler(this.jpegHdr);
    this.onFrame = onFrame;
    this.streaming = true;
    // both loops run until stop() clears this.streaming; stop() awaits them
    this.isoDone = this.isoLoop(depth, packets);
    this.brcDone = this.brcLoop();
    return { width, height };
  }

  /**
   * Keep `depth` isochronous transfers in flight, retiring them in submission
   * order so packets reach the assembler in the order the bridge sent them.
   */
  async isoLoop(depth, packets) {
    const lengths = new Array(packets).fill(this.packetSize);
    const inflight = [];
    const submit = () => {
      const transfer = this.device.isochronousTransferIn(EP_VIDEO, lengths);
      // keep unhandled-rejection warnings quiet for the transfers we abandon
      // on error; the copy we await still sees the rejection
      transfer.catch(() => {});
      return transfer;
    };

    try {
      for (let i = 0; i < depth; i++) inflight.push(submit());
      while (this.streaming) {
        const result = await inflight.shift();
        if (this.streaming) inflight.push(submit());
        for (const packet of result.packets) {
          if (packet.status !== 'ok' || !packet.data || !packet.data.byteLength) {
            continue;
          }
          const data = new Uint8Array(packet.data.buffer,
                                      packet.data.byteOffset,
                                      packet.data.byteLength);
          const frame = this.assembler.feed(data);
          if (frame && this.onFrame) this.onFrame(frame);
        }
      }
    } catch (error) {
      if (this.streaming) {
        this.streaming = false;
        const explained = explainIsoFailure(error);
        this.log(`iso transfer failed: ${explained.message}`);
        if (this.onError) this.onError(explained);
      }
    } finally {
      await Promise.allSettled(inflight);
    }
  }

  /**
   * The bridge's bit-rate-control loop.  Bit 0 of register 0x11 flags a FIFO
   * overflow: tighten the allowed bytes per isochronous packet on overflow and
   * back off again after ten clean polls, like the kernel driver's work queue.
   */
  async brcLoop() {
    while (this.streaming) {
      await sleep(100);
      if (!this.streaming) break;
      let change;
      try {
        change = (await this.regR(0x0011)) & 0x01;
      } catch (error) {
        break;
      }
      if (change) {                         // overflow
        this.brcGood = 0;
        if (this.reg07 === 0) this.reg07 = 0x32;   // allow 98 bytes/unit
        else if (this.reg07 > 2) this.reg07 -= 2;
        else change = 0;
      } else {
        this.brcGood++;
        if (this.brcGood >= 10) {
          this.brcGood = 0;
          if (this.reg07) {
            change = 1;
            if (this.reg07 < 0x32) this.reg07 += 2;
            else this.reg07 = 0;
          }
        }
      }
      if (change) {
        try {
          await this.regW(this.reg07, 0x0007);
        } catch (error) {
          break;
        }
      }
    }
  }

  async stop() {
    if (!this.streaming) return;
    this.streaming = false;
    // wait for the in-flight isochronous transfers to retire before touching
    // the alternate setting, otherwise Chrome fails them
    await Promise.allSettled([this.isoDone, this.brcDone]);
    try {
      await this.device.selectAlternateInterface(IFACE, 0);
      await this.sendUnknown(this.sensor);
      await this.regW(0x01, 0x0000);        // led off
    } catch (error) {
      this.log(`stop: ${error}`);
    }
  }
}

// ---------------------------------------------------------------------------
// device access
// ---------------------------------------------------------------------------
export async function requestCamera() {
  return navigator.usb.requestDevice({
    filters: PRODUCT_IDS.map((productId) => ({ vendorId: VENDOR_ID, productId })),
  });
}

export async function getPairedCamera() {
  const devices = await navigator.usb.getDevices();
  return devices.find((d) => d.vendorId === VENDOR_ID &&
                             PRODUCT_IDS.includes(d.productId)) || null;
}

/** Open, configure and probe a device returned by requestDevice/getDevices. */
export async function openCamera(device, options = {}) {
  await device.open();
  if (device.configuration === null) await device.selectConfiguration(1);
  await device.claimInterface(IFACE);
  const camera = new Camera(device, options);
  await camera.init(options.forceSensor || null);
  return camera;
}

export async function closeCamera(camera) {
  await camera.stop();
  try {
    await camera.device.releaseInterface(IFACE);
  } catch (error) { /* already gone */ }
  try {
    await camera.device.close();
  } catch (error) { /* already gone */ }
}
