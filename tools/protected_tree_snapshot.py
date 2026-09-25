"""Read-only content audit for third_party trees; outputs live outside them."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def snapshot(roots):
    result = {}
    for root in roots:
        root = root.resolve(strict=True)
        files = {}
        for directory, dirs, names in os.walk(root):
            dirs.sort()
            for name in sorted(names):
                path = Path(directory) / name
                with path.open('rb') as stream:
                    digest = hashlib.file_digest(stream, 'sha256').hexdigest()
                files[path.relative_to(root).as_posix()] = digest
        result[str(root)] = files
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', action='append', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--compare', type=Path)
    args = p.parse_args()
    for root in args.root:
        if args.output.resolve().is_relative_to(root.resolve()):
            p.error('audit output must be outside protected roots')
    result = snapshot(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    if args.compare:
        before = json.loads(args.compare.read_text(encoding='utf-8'))
        changes = {root: sorted(k for k in set(before.get(root, {})) | set(files)
                                if before.get(root, {}).get(k) != files.get(k))
                   for root, files in result.items()}
        print(json.dumps({'unchanged': before == result, 'changes': changes}))
        return 0 if before == result else 1
    print(json.dumps({'files': sum(map(len, result.values())), 'output': str(args.output)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
