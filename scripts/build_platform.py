# SPDX-License-Identifier: Apache-2.0
"""Build the reviewed r1 locally from user-supplied or official CAD files."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
FILES = {
    'Wrist_Roll_08c.stl': ('https://tna001-ai.github.io/LeFlexiTac/assets/models/Wrist_Roll_08c.stl', '7e78b577cd03f5c3e1ec5c2a5c1f0fb71cbaed23e6dd2bf76dc166a3b0b7bdeb'),
    'Moving_Jaw_08d.stl': ('https://tna001-ai.github.io/LeFlexiTac/assets/models/Moving_Jaw_08d.stl', 'c0f71e0134a7cd3a425488ba5cab87e16ee677f4768cd57036d21b9b81c6ea60'),
}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--wrist', type=Path, help='local official Wrist_Roll_08c.stl')
    p.add_argument('--jaw', type=Path, help='local official Moving_Jaw_08d.stl')
    p.add_argument('--download', action='store_true', help='download missing inputs directly from the official LeFlexiTac site')
    p.add_argument('--output_dir', type=Path, default=Path('.local/platform_build'), help='new local-only output directory (must not exist)')
    a = p.parse_args()
    output = a.output_dir.resolve()
    if output.exists():
        p.error(f'output already exists: {output}; choose a fresh directory')
    if not a.download and (not a.wrist or not a.jaw):
        p.error('provide --wrist and --jaw, or use --download')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Stage on the destination filesystem; no partial platform is exposed on failure.
    with tempfile.TemporaryDirectory(prefix='.so101-build-', dir=output.parent) as tmp:
        stage = Path(tmp) / 'platform'
        assets = stage / 'assets'
        assets.mkdir(parents=True)
        for (name, (url, expected)), supplied in zip(FILES.items(), (a.wrist, a.jaw)):
            target = assets / name
            if supplied:
                data = supplied.read_bytes()
            else:
                with urllib.request.urlopen(url, timeout=60) as response:
                    data = response.read(20 * 1024 * 1024 + 1)
                if len(data) > 20 * 1024 * 1024:
                    raise ValueError(f'{name} exceeds download limit')
            if hashlib.sha256(data).hexdigest() != expected:
                raise ValueError(f'{name}: SHA-256 mismatch; upstream CAD changed or is corrupted')
            target.write_bytes(data)
        env = os.environ.copy()
        env['PYTHONPATH'] = str(ROOT / 'src') + os.pathsep + env.get('PYTHONPATH', '')
        def run(script, *args):
            subprocess.run([sys.executable, str(ROOT / 'scripts' / script), *map(str, args)], check=True, env=env)
        revision = stage / 'revision'
        run('restore_hex_mount.py', '--wrist', assets / 'Wrist_Roll_08c.stl', '--output_dir', revision)
        cad = revision / 'Wrist_Roll_08c_HexNut_M3_r1.stl'
        run('build_robot.py', '--local_assets', assets, '--wrist_override', cad, '--output_dir', stage / 'robot', '--camera', 'true')
        robot = stage / 'robot' / 'robot.usda'
        reference = json.loads((ROOT / 'docs' / 'reviewed_r1.json').read_text())
        for path, key in ((cad, 'cad'), (robot, 'robot')):
            if sha(path) != reference['reviewed_artifacts'][key]:
                raise ValueError(f'{key} output differs from the reviewed artifact; cannot claim reproduction. Check pinned environment.')
        # A historical report, tied to exact hashes; this does not claim a new local run.
        review = stage / 'historical_review.json'
        review.write_text(json.dumps(reference, indent=2) + '\n')
        manifest = {'schema': 'so101-flexitac-local-platform-v1', 'variant': 'Wrist_Roll_08c_HexNut_M3_r1', 'root_prim': '/Robot', 'joint_root': 'Joints', 'fixed_links': {'CameraMount': 'gripper_link', 'CameraBody': 'CameraMount'}, 'hardware_validation': 'NOT_PERFORMED', 'redistribution': 'LOCAL_ONLY_LEFLEXITAC_PERMISSION_UNCONFIRMED', 'verification_scope': 'historical numerical run; reproduced artifact hashes; not a new local simulation run'}
        for key, path in (('robot', robot), ('cad', cad), ('verification', review)):
            manifest[key] = {'path': str(path.relative_to(stage)), 'sha256': sha(path)}
        (stage / 'platform.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(output)
    print(f'Local-only platform: {output}')
    print(f'Set SO101_FLEXITAC_PLATFORM_MANIFEST={output / "platform.json"}')
    print('Hardware assembly has not been verified. Do not redistribute generated CAD or USD.')

if __name__ == '__main__':
    main()
