"""Reject UF2 images containing SoftDevice, bootloader, settings or UICR writes."""
import hashlib
import json
import re
import struct
from pathlib import Path

root = Path(__file__).resolve().parents[1]
report = {}
for path in sorted((root / 'dist').glob('*.uf2')):
    data = path.read_bytes()
    assert len(data) % 512 == 0, path
    blocks = {}
    numbers = set()
    for off in range(0, len(data), 512):
        block = data[off:off + 512]
        magic0, magic1, flags, addr, size, num, count, family = struct.unpack_from('<8I', block)
        assert (magic0, magic1) == (0x0A324655, 0x9E5D5157)
        assert struct.unpack_from('<I', block, 508)[0] == 0x0AB16F30
        assert flags == 0x2000 and family == 0xADA52840, (path, flags, family)
        assert size == 256 and addr % 256 == 0
        assert 0x26000 <= addr and addr + size <= 0xEC000, (path, hex(addr))
        assert count == len(data) // 512 and num not in numbers
        assert addr not in blocks
        numbers.add(num)
        blocks[addr] = block[32:32 + size]
    assert numbers == set(range(len(blocks)))
    assert min(blocks) == 0x26000
    sp, reset = struct.unpack_from('<II', blocks[0x26000])
    assert 0x20000000 < sp <= 0x20040000 and sp % 8 == 0, hex(sp)
    assert reset & 1 and 0x26000 <= reset < 0xEC000, hex(reset)
    report[path.name] = {
        'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data), 'blocks': len(blocks),
        'flash_start': hex(min(blocks)), 'flash_end_exclusive': hex(max(blocks) + 256),
        'initial_sp': hex(sp), 'reset_handler': hex(reset), 'uf2_family': hex(family),
        'application_only': True, 'hardware_tested': False,
    }
    print(f'PASS: {path.name}: {hex(min(blocks))}..{hex(max(blocks) + 256)}; application only')
assert report, 'No UF2 files found'
for image in report:
    variant = Path(image).stem
    build = root / 'build' / variant / 'zephyr'
    config = (build / '.config').read_text()
    diagnostic = 'diagnostic' in variant
    clock = 'XTAL'
    for setting in ('KSCAN_PREC_EC', 'BT_CTLR_TX_PWR_0', 'ZMK_USB', 'ZMK_BLE',
                    f'CLOCK_CONTROL_NRF_K32SRC_{clock}', 'CLOCK_CONTROL_NRF_K32SRC_500PPM',
                    'RETENTION_BOOT_MODE', 'ZMK_BOARD_COMPAT'):
        assert f'CONFIG_{setting}=y\n' in config, (variant, setting)
    # The shell owns USB CDC in diagnostic images; ZMK Studio owns it everywhere else.
    assert ('CONFIG_ZMK_STUDIO=y\n' in config) == (not diagnostic), (variant, 'studio')
    assert 'CONFIG_GPIO_AS_PINRESET=y\n' not in config, (variant, 'use &uicr gpio-as-nreset')
    assert ('CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC_CALIBRATION=y\n' in config) == (clock == 'RC')
    other_clock = 'RC' if clock == 'XTAL' else 'XTAL'
    assert f'CONFIG_CLOCK_CONTROL_NRF_K32SRC_{other_clock}=y\n' not in config
    assert 'CONFIG_ADC_NRFX_SAADC=y\n' not in config
    assert 'CONFIG_NRFX_TIMER2=y\n' not in config
    assert 'CONFIG_ZMK_SLEEP=y\n' not in config
    assert ('CONFIG_SHELL=y\n' in config) == diagnostic
    assert ('CONFIG_PREC_EC_REPORT_KEYS=y\n' in config) == (not diagnostic)
    assert ('CONFIG_PREC_EC_AUTO_SCAN=y\n' in config) == (not diagnostic)
    if variant.startswith('diel-power-save'):
        assert 'CONFIG_PREC_EC_KEY_STANDBY=y\n' in config
        for name, value in {'PREC_EC_SAMPLE_DELAY_US':40, 'PREC_EC_PRESS_DELTA':500,
                            'PREC_EC_RELEASE_DELTA':300, 'PREC_EC_ACTIVE_PERIOD_MS':5,
                            'PREC_EC_IDLE_PERIOD_MS':20, 'PREC_EC_STANDBY_TIMEOUT_MS':300000,
                            'PREC_EC_STANDBY_PERIOD_MS':50, 'PREC_EC_STANDBY_HOLD_MS':300,
                            'PREC_EC_STANDBY_ROW':3, 'PREC_EC_STANDBY_COL':5}.items():
            assert f'CONFIG_{name}={value}\n' in config, (variant, name)
    else:
        assert 'CONFIG_PREC_EC_KEY_STANDBY=y\n' not in config, (variant, 'standby must be opt-in')
    board_name = 'diel'
    assert f'CONFIG_BOARD="{board_name}"\n' in config
    assert f'CONFIG_BOARD_{board_name.upper()}_NRF52840_ZMK=y\n' in config, (variant, 'zmk variant')
    dts = (build / 'zephyr.dts').read_text()
    scan_node = re.search(r'ec_scan:\s+ec_scan\s*\{([^}]+)\}', dts, re.DOTALL)[1]
    assert 'discharge-active-high;' in scan_node, (variant, 'discharge polarity')
    chosen = re.search(r'\bchosen\s*\{([^}]+)\}', dts, re.DOTALL)[1]
    assert f'zmk,physical-layout = &{board_name}_layout;' in chosen, (variant, 'physical layout')
    assert 'zmk,matrix-transform' not in chosen, (variant, 'Studio requires no chosen transform')
    layouts = re.findall(r'compatible = "zmk,physical-layout";', dts)
    assert len(layouts) == 1, (variant, 'exactly one selectable physical layout')
    uicr = re.search(r'uicr:\s+uicr@[^{}]+\{([^}]+)\}', dts, re.DOTALL)[1]
    assert 'gpio-as-nreset;' in uicr, (variant, 'SW301 reset pin')
    reg1 = re.search(r'reg1:\s+regulator@[^{}]+\{([^}]+)\}', dts, re.DOTALL)[1]
    mode = re.search(r'regulator-initial-mode\s*=\s*<\s*(\w+)\s*>', reg1)[1]
    assert int(mode, 0) == 1, (variant, 'REG1 must use DCDC mode')
    print(f'PASS: {variant} board/discharge polarity, ADC/TIMER ownership, USB/BLE/clock, layout/Studio')
(root / 'dist/verification.json').write_text(json.dumps(report, indent=2) + '\n')
