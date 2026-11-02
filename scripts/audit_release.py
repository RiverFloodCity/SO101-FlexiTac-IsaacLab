# SPDX-License-Identifier: Apache-2.0
"""Build an explicit release allowlist; fail on unknown files, secrets or private dependencies."""

import argparse
import hashlib
import json
import os
import re
import zipfile
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--output_dir", type=Path, required=True)
a = p.parse_args()
r = Path(__file__).resolve().parents[1]
a.output_dir.mkdir(parents=True, exist_ok=True)
allowed_roots = {"src", "scripts", "tests", "docs", "licenses"}
allowed_top = {
    "README.md",
    "LICENSE",
    "THIRD_PARTY_NOTICES.md",
    "CITATION.cff",
    "pyproject.toml",
    ".gitignore",
    "requirements-sim.lock",
    "constraints-sim.txt",
    "RELEASE_FILES.json",
}
files = []
problems = []
blocked_hashes = {
    x["sha256"]
    for x in json.loads((r / "docs/source_manifest.json").read_text())
    if not x["redistribute"]
}
candidates = []
for directory, folders, names in os.walk(r, followlinks=False):
    folders[:] = [
        name
        for name in folders
        if name
        not in {
            ".git",
            ".local",
            ".venv",
            "__pycache__",
            ".pytest_cache",
            ".ruff_cache",
            "build",
            "dist",
        }
        and not name.endswith(".egg-info")
    ]
    for name in names:
        if name != "RELEASE_FILES.json":
            candidates.append(Path(directory) / name)
for path in sorted(candidates):
    rel = path.relative_to(r)
    if any(
        t in {".git", ".local", ".venv", "__pycache__", ".pytest_cache", "build", "dist"}
        or t.endswith(".egg-info")
        for t in rel.parts
    ):
        continue
    if not path.is_file():
        continue
    if rel.parts[0] not in allowed_roots and str(rel) not in allowed_top:
        problems.append(f"Unknown release path: {rel}")
        continue
    if path.is_symlink():
        problems.append(f"Symlink forbidden: {rel}")
        continue
    data = path.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    if sha in blocked_hashes:
        problems.append(f"Unlicensed binary: {rel}")
    if path.suffix in {".usd", ".usda", ".usdc"}:
        problems.append(f"Generated local derivative in public scope: {rel}")
    try:
        text = data.decode()
    except UnicodeDecodeError:
        text = ""
    patterns = [
        r"/" + "home/" + r"[^\s\"']+",
        r"(?:sk-proj-|sk-live-|ghp_|github_pat_)[A-Za-z0-9_]{16,}",
        r"(?:from|import)\s+simtacgen(?:[.\s]|$)",
        r"BEGIN (?:RSA |OPENSSH )?PRIVATE KEY",
    ]
    for pattern in patterns:
        if re.search(pattern, text):
            problems.append(f"Sensitive/private pattern in {rel}")
    files.append(dict(path=str(rel), sha256=sha, bytes=len(data)))
for item in json.loads((r / "docs/source_manifest.json").read_text()):
    if item["redistribute"]:
        path = r / item["path"]
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            problems.append("Borrowed file changed without audit: " + item["path"])
if not problems:
    listing = r / "RELEASE_FILES.json"
    listing.write_text(
        json.dumps([item["path"] for item in files] + ["RELEASE_FILES.json"], indent=2)
    )
    data = listing.read_bytes()
    files.append(
        dict(path="RELEASE_FILES.json", sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))
    )
report = dict(
    status="PASS" if not problems else "FAIL",
    files=files,
    problems=problems,
    exclusions=[
        ".local (all custom CAD, derivatives, operator rest and render evidence)",
        ".venv",
        ".git",
        "caches",
        "private datasets and task source",
    ],
    disclaimer="File allowlist clearance is not physical validation or permission to redistribute missing-license CAD.",
)
(a.output_dir / "release_audit.json").write_text(json.dumps(report, indent=2))
if problems:
    raise SystemExit("\n".join(problems))
with zipfile.ZipFile(a.output_dir / "public_candidate.zip", "w", zipfile.ZIP_DEFLATED) as z:
    for item in files:
        if item["path"] != "RELEASE_FILES.json":
            z.write(r / item["path"], item["path"])
    z.write(r / "RELEASE_FILES.json", "RELEASE_FILES.json")
print(
    json.dumps(
        {
            "status": report["status"],
            "files": len(files),
            "archive": str(a.output_dir / "public_candidate.zip"),
        }
    )
)
