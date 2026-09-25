"""Make a fresh test SDK without legacy custom SensorHealth types or master.

Copies an installed binary SDK; does not rebuild core libraries or change sources.
Records a file manifest so the provenance and exclusion can be audited.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    source, output = a.source.resolve(strict=True), a.output.resolve()
    if output.exists() or output.is_relative_to(source) or 'third_party' in output.parts:
        p.error('output must be a NEW directory outside source and third_party')
    excluded = []
    def ignore(directory, names):
        skip = [n for n in names if 'SensorHealth' in n or n == '__pycache__'
                or n.lower() in ('hako-master.exe', 'hako-master') or n.endswith('.pyc')]
        excluded.extend(str((Path(directory) / n).relative_to(source)) for n in skip)
        return skip
    shutil.copytree(source, output, ignore=ignore)
    for required in ('include/hakoniwa/pdu/std_msgs/pdu_cpptype_UInt8MultiArray.hpp',
                     'python/hakoniwa_pdu/pdu_msgs/std_msgs/pdu_conv_UInt8MultiArray.py'):
        if not (output / required).is_file():
            raise RuntimeError('Standard SDK type missing: ' + required)
    files = {}
    for file in sorted(output.rglob('*')):
        if file.is_file():
            files[file.relative_to(output).as_posix()] = hashlib.sha256(file.read_bytes()).hexdigest()
    report = {'source': str(source), 'output': str(output), 'excluded': excluded,
              'kind': 'isolated binary SDK copy; core libraries reused', 'sha256': files}
    (output / 'sens006_sdk_manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'copied_files': len(files), 'excluded': len(excluded), 'output': str(output)}))


if __name__ == '__main__':
    main()
