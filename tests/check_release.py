#!/usr/bin/env python3
"""Проверка целостности исходного релиза. Не заменяет компиляцию ESP32."""
from pathlib import Path
import hashlib, json, re
root = Path(__file__).resolve().parents[1]
assert (root / 'VERSION').read_text().strip() == '1.2.0'
sketch = root / 'ClimateCamera_1_2_0_IDE1819'
required = ['ClimateCamera_1_2_0_IDE1819.ino', 'ClimateCamera.h', 'ClimateCamera.cpp',
            'FirmwareVersion.h', 'NetworkServer.h', 'NetworkServer.cpp', 'WebUI.h',
            'NetworkConfig.h', 'Secrets.example.h', 'Sht30Reliable.h', 'Sht30Reliable.cpp', 'User_Setup.h']
for name in required:
    assert (sketch / name).is_file() and (sketch / name).stat().st_size > 0, name
assert '#define FW_VERSION "1.2.0"' in (sketch / 'FirmwareVersion.h').read_text()
assert (sketch / 'User_Setup.h').read_bytes() == (root / 'TFT_eSPI_setup/User_Setup.h').read_bytes()
assert not list(root.rglob('*.bin')), 'Архив исходников не должен содержать старые BIN'
assert not list(root.rglob('Secrets.h')), 'Не публикуйте личный Secrets.h'
checked = set()
for line in (root / 'SHA256SUMS.txt').read_text().splitlines():
    digest, filename = line.split('  ', 1)
    assert re.fullmatch(r'[0-9a-f]{64}', digest), filename
    rel = Path(filename)
    assert not rel.is_absolute() and '..' not in rel.parts, filename
    assert filename not in checked, filename
    path = root / rel
    assert path.is_file(), filename
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, filename
    checked.add(filename)
expected = {str(p.relative_to(root)).replace('\\','/') for p in root.rglob('*')
            if p.is_file() and p.name != 'SHA256SUMS.txt'}
assert checked == expected, 'Неполное покрытие контрольными суммами'
manifest = json.loads((root / 'release_manifest.json').read_text())
assert manifest['version'] == '1.2.0'
assert manifest['display'] == 'ST7789 240x240'
assert manifest['artifact_type'] == 'source_release'
assert manifest['esp32_compilation_performed'] is False
print(f'OK: v1.2.0; {len(checked)} SHA-256 checks; sketch and TFT profiles consistent')
