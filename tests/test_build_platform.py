# SPDX-License-Identifier: Apache-2.0
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/build_platform.py"

def test_rejects_changed_inputs_and_leaves_no_output(tmp_path):
    source = tmp_path / "incorrect.stl"
    source.write_bytes(b"incorrect revision")
    out = tmp_path / "platform"
    result = subprocess.run([sys.executable, str(SCRIPT), "--wrist", str(source), "--jaw", str(source), "--output_dir", str(out)], capture_output=True, text=True)
    assert result.returncode != 0
    assert "SHA-256 mismatch" in result.stderr
    assert not out.exists()

def test_existing_output_is_never_overwritten(tmp_path):
    out = tmp_path / "platform"
    out.mkdir()
    marker = out / "marker"
    marker.write_text("preserve")
    result = subprocess.run([sys.executable, str(SCRIPT), "--download", "--output_dir", str(out)], capture_output=True, text=True)
    assert result.returncode != 0
    assert "already exists" in result.stderr
    assert marker.read_text() == "preserve"
