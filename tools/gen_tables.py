#!/usr/bin/env python3
"""Extract the zc3xx register/init tables from the Linux gspca driver sources.

The gspca zc3xx driver is ~7000 lines, almost all of it machine-readable
tables of USB control exchanges.  Rather than transcribing them by hand we
parse ref/zc3xx.c, ref/zc3xx-reg.h and ref/jpeg.h and emit zc3xx/_tables.py.

Usage: python3 tools/gen_tables.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF = os.path.join(ROOT, 'ref')
OUT = os.path.join(ROOT, 'zc3xx', '_tables.py')
OUT_JS = os.path.join(ROOT, 'webusb', 'zc3xx-tables.js')


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    text = re.sub(r'//[^\n]*', ' ', text)
    return text


# --------------------------------------------------------------------------
# a very small parser for C brace initialisers
# --------------------------------------------------------------------------
TOKEN = re.compile(r'\s*(\{|\}|,|\[[^\]]*\]\s*=|[A-Za-z_][A-Za-z_0-9]*|0[xX][0-9a-fA-F]+|-?\d+)')


def tokenize(text):
    pos = 0
    out = []
    while pos < len(text):
        m = TOKEN.match(text, pos)
        if not m:
            if text[pos].isspace():
                pos += 1
                continue
            raise ValueError('unexpected %r at %d' % (text[pos:pos + 20], pos))
        out.append(m.group(1).strip())
        pos = m.end()
    return out


def parse_initializer(tokens, i, resolve):
    """Parse a {...} starting at tokens[i] == '{'.  Returns (value, next_i).

    Designated entries ("[SENSOR_X] = ...") come back as a dict, plain ones
    as a list.
    """
    assert tokens[i] == '{', tokens[i]
    i += 1
    items = []
    named = {}
    key = None
    while tokens[i] != '}':
        tok = tokens[i]
        if tok == ',':
            i += 1
            continue
        if tok.startswith('['):
            key = tok[1:tok.index(']')].strip()
            i += 1
            continue
        if tok == '{':
            val, i = parse_initializer(tokens, i, resolve)
        else:
            val = resolve(tok)
            i += 1
        if key is not None:
            named[key] = val
            key = None
        else:
            items.append(val)
    return (named if named else items), i + 1


def find_array(src, decl_re):
    """Return the token list of the initialiser for the first matching decl."""
    m = re.search(decl_re, src)
    if not m:
        raise KeyError(decl_re)
    start = src.index('{', m.end() - 1)
    depth = 0
    for j in range(start, len(src)):
        if src[j] == '{':
            depth += 1
        elif src[j] == '}':
            depth -= 1
            if depth == 0:
                return src[start:j + 1]
    raise ValueError('unbalanced braces for %s' % decl_re)


def main():
    regs = {}
    reg_src = open(os.path.join(REF, 'zc3xx-reg.h')).read()
    for m in re.finditer(r'#define\s+(ZC3XX_R\w+)\s+(0x[0-9a-fA-F]+)', reg_src):
        regs[m.group(1)] = int(m.group(2), 16)

    src = strip_comments(open(os.path.join(REF, 'zc3xx.c')).read())

    def num(tok):
        if tok in regs:
            return regs[tok]
        try:
            return int(tok, 0)
        except ValueError:
            return tok  # a symbol (NULL, table name, SENSOR_x ...)

    # ---- struct usb_action tables -----------------------------------------
    actions = {}
    for m in re.finditer(
            r'static const struct usb_action\s+(\w+)\s*\[\]\s*=\s*', src):
        name = m.group(1)
        body = find_array(src, re.escape(m.group(0)))
        seq = []
        for entry in re.finditer(r'\{([^{}]*)\}', body[1:-1]):
            parts = [p.strip() for p in entry.group(1).split(',') if p.strip()]
            if not parts:
                continue            # the {} terminator
            if len(parts) != 3:
                raise ValueError('%s: bad entry %r' % (name, parts))
            req, val, idx = (num(p) for p in parts)
            for v in (req, val, idx):
                if not isinstance(v, int):
                    raise ValueError('%s: unresolved %r' % (name, v))
            seq.append((req, val, idx))
        actions[name] = seq

    # ---- sensor enum -------------------------------------------------------
    enum_body = find_array(src, r'enum sensors\s*')
    sensors = [t for t in re.split(r'[,\s]+', enum_body.strip('{} \n')) if t]
    sensors = [s for s in sensors if s != 'SENSOR_MAX']

    def designated(decl_re, mapper=lambda v: v):
        body = find_array(src, decl_re)
        val, _ = parse_initializer(tokenize(body), 0, num)
        assert isinstance(val, dict), decl_re
        return {k: mapper(v) for k, v in val.items()}

    init_tb = designated(r'static const struct usb_action \*init_tb\[SENSOR_MAX\]\[2\]\s*=\s*')
    freq_tb = designated(r'static const struct usb_action \*freq_tb\[SENSOR_MAX\]\[6\]\s*=\s*')
    mode_tb = designated(r'static const u8 mode_tb\[SENSOR_MAX\]\s*=\s*')
    gamma_def = designated(r'static const u8 gamma\[SENSOR_MAX\]\s*=\s*')
    matrix_tb = designated(r'static const u8 \*matrix_tb\[SENSOR_MAX\]\s*=\s*')

    # ---- plain u8 arrays ---------------------------------------------------
    plain = {}
    for name, decl in (
            ('adcm2700_matrix', r'static const u8 adcm2700_matrix\[9\]\s*=\s*'),
            ('gc0305_matrix', r'static const u8 gc0305_matrix\[9\]\s*=\s*'),
            ('ov7620_matrix', r'static const u8 ov7620_matrix\[9\]\s*=\s*'),
            ('pas202b_matrix', r'static const u8 pas202b_matrix\[9\]\s*=\s*'),
            ('po2030_matrix', r'static const u8 po2030_matrix\[9\]\s*=\s*'),
            ('tas5130c_matrix', r'static const u8 tas5130c_matrix\[9\]\s*=\s*'),
            ('gc0303_matrix', r'static const u8 gc0303_matrix\[9\]\s*=\s*'),
            ('delta_b', r'static const u8 delta_b\[16\]\s*=\s*'),
            ('delta_c', r'static const u8 delta_c\[16\]\s*=\s*'),
            ('gamma_tb', r'static const u8 gamma_tb\[6\]\[16\]\s*=\s*'),
            ('sharpness_tb', r'static const u8 sharpness_tb\[\]\[2\]\s*=\s*'),
            ('jpeg_qual', r'static u8 jpeg_qual\[\]\s*=\s*'),
    ):
        body = find_array(src, decl)
        val, _ = parse_initializer(tokenize(body), 0, num)
        plain[name] = val

    # chipset revision -> internal sensor id
    body = find_array(
        src, r'static const struct sensor_by_chipset_revision\s+chipset_revision_sensor\[\]\s*=\s*')
    chip_rev = []
    for entry in re.finditer(r'\{\s*(0x[0-9a-fA-F]+)\s*,\s*(0x[0-9a-fA-F]+)\s*\}', body):
        chip_rev.append((int(entry.group(1), 16), int(entry.group(2), 16)))

    # ---- v4l2 mode tables --------------------------------------------------
    modes = {}
    for name in ('vga_mode', 'broken_vga_mode', 'sif_mode'):
        body = find_array(src, r'static const struct v4l2_pix_format %s\[\]\s*=\s*' % name)
        entries = []
        for entry in re.finditer(r'\{\s*(\d+)\s*,\s*(\d+)\s*,.*?\.priv\s*=\s*(\d+)', body, re.S):
            entries.append((int(entry.group(1)), int(entry.group(2)), int(entry.group(3))))
        modes[name] = entries

    # ---- JPEG header from jpeg.h ------------------------------------------
    jsrc = open(os.path.join(REF, 'jpeg.h')).read()
    hdr_off = {}
    for m in re.finditer(r'#define\s+(JPEG_\w+)\s+(\d+)', jsrc):
        hdr_off[m.group(1)] = int(m.group(2))
    jbody = find_array(strip_comments(jsrc), r'static const u8 jpeg_head\[\]\s*=\s*')
    jbody = re.sub(r'^\s*#.*$', '', jbody, flags=re.M)      # drop #ifdef/#define
    jpeg_head = [int(t, 0) for t in re.findall(r'0[xX][0-9a-fA-F]+|\b\d+\b', jbody)]
    if len(jpeg_head) != hdr_off['JPEG_HDR_SZ']:
        raise ValueError('jpeg_head is %d bytes, expected %d'
                         % (len(jpeg_head), hdr_off['JPEG_HDR_SZ']))

    # ---- emit --------------------------------------------------------------
    with open(OUT, 'w') as f:
        w = f.write
        w('"""Generated by tools/gen_tables.py from the Linux gspca zc3xx driver.\n\n'
          'Do not edit by hand.  Source: drivers/media/usb/gspca/{zc3xx.c,zc3xx-reg.h,jpeg.h}\n'
          '(GPL-2.0-or-later, Jean-Francois Moine / Michel Xhaard).\n"""\n\n')
        w('SENSORS = %r\n\n' % (sensors,))
        w('REGS = {\n')
        for k in sorted(regs, key=lambda n: (regs[n], n)):
            w('    %r: 0x%04x,\n' % (k, regs[k]))
        w('}\n\n')
        w('MODES = %r\n\n' % (modes,))
        w('MODE_TB = %r\n\n' % (mode_tb,))
        w('GAMMA_DEF = %r\n\n' % (gamma_def,))
        w('MATRIX_TB = %r\n\n' % (matrix_tb,))
        w('INIT_TB = %r\n\n' % (init_tb,))
        w('FREQ_TB = %r\n\n' % (freq_tb,))
        w('CHIP_REVISION_SENSOR = %r\n\n' % (chip_rev,))
        for k in sorted(plain):
            w('%s = %r\n\n' % (k.upper(), plain[k]))
        w('JPEG_QT0_OFFSET = %d\n' % hdr_off['JPEG_QT0_OFFSET'])
        w('JPEG_QT1_OFFSET = %d\n' % hdr_off['JPEG_QT1_OFFSET'])
        w('JPEG_HEIGHT_OFFSET = %d\n' % hdr_off['JPEG_HEIGHT_OFFSET'])
        w('JPEG_HDR_SZ = %d\n\n' % hdr_off['JPEG_HDR_SZ'])
        w('JPEG_HEAD = bytes(%r)\n\n' % (bytes(jpeg_head),))
        w('ACTIONS = {\n')
        for name in sorted(actions):
            w('    %r: (\n' % name)
            for req, val, idx in actions[name]:
                w('        (0x%02x, 0x%02x, 0x%04x),\n' % (req, val, idx))
            w('    ),\n')
        w('}\n')

    # ---- the same tables as an ES module, for the WebUSB build -----------
    os.makedirs(os.path.dirname(OUT_JS), exist_ok=True)
    with open(OUT_JS, 'w') as f:
        w = f.write
        w('// Generated by tools/gen_tables.py from the Linux gspca zc3xx driver.\n'
          '// Do not edit by hand.  Source: drivers/media/usb/gspca/zc3xx.c\n'
          '// (GPL-2.0-or-later, Jean-Francois Moine / Michel Xhaard).\n'
          '//\n'
          '// ACTIONS values are packed hex: 2 chars request, 2 value, 4 index.\n\n')

        def js(value):
            import json
            return json.dumps(value)

        w('export const SENSORS = %s;\n\n' % js(sensors))
        w('export const MODES = %s;\n\n' % js(
            {k: [list(e) for e in v] for k, v in modes.items()}))
        w('export const MODE_TB = %s;\n\n' % js(mode_tb))
        w('export const GAMMA_DEF = %s;\n\n' % js(gamma_def))
        w('export const MATRIX_TB = %s;\n\n' % js(matrix_tb))
        w('export const INIT_TB = %s;\n\n' % js(init_tb))
        w('export const FREQ_TB = %s;\n\n' % js(freq_tb))
        w('export const CHIP_REVISION_SENSOR = %s;\n\n' % js(
            [list(e) for e in chip_rev]))
        for k in sorted(plain):
            w('export const %s = %s;\n\n' % (k.upper(), js(plain[k])))
        w('export const JPEG_QT0_OFFSET = %d;\n' % hdr_off['JPEG_QT0_OFFSET'])
        w('export const JPEG_QT1_OFFSET = %d;\n' % hdr_off['JPEG_QT1_OFFSET'])
        w('export const JPEG_HEIGHT_OFFSET = %d;\n' % hdr_off['JPEG_HEIGHT_OFFSET'])
        w('export const JPEG_HDR_SZ = %d;\n\n' % hdr_off['JPEG_HDR_SZ'])
        w('export const JPEG_HEAD = Uint8Array.from(atob(%s), c => c.charCodeAt(0));\n\n'
          % js(__import__('base64').b64encode(bytes(jpeg_head)).decode()))
        w('export const ACTIONS = {\n')
        for name in sorted(actions):
            packed = ''.join('%02x%02x%04x' % entry for entry in actions[name])
            w('  %s: "%s",\n' % (js(name), packed))
        w('};\n')

    print('wrote %s' % OUT_JS)
    print('wrote %s' % OUT)
    print('  %d usb_action tables, %d entries total'
          % (len(actions), sum(len(v) for v in actions.values())))
    print('  %d sensors, jpeg header %d bytes' % (len(sensors), len(jpeg_head)))


if __name__ == '__main__':
    sys.exit(main())
