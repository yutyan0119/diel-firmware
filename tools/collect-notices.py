"""Keep license notices and exact source locations for fetched west projects."""
import json
from pathlib import Path
import re
from subprocess import check_output
import sys

workspace, output = (Path(arg).resolve() for arg in sys.argv[1:])
output.mkdir(parents=True, exist_ok=True)
projects = check_output(
    ["west", "list", "-f", "{name}|{abspath}|{url}"], cwd=workspace, text=True
)
notice_name = re.compile(r"(?:licen[cs]es?|copying|notice|copyright)(?:[._-].*)?$", re.I)
manifest = []
for line in projects.splitlines():
    name, path, url = line.split("|", 2)
    project = Path(path)
    if url == "None" or not (project / ".git").exists():
        continue
    revision = check_output(["git", "rev-parse", "HEAD"], cwd=project, text=True).strip()
    manifest.append({"project": name, "url": url, "revision": revision})
    for rel in check_output(["git", "ls-files", "-z"], cwd=project).decode().split("\0"):
        source = project / rel
        if notice_name.fullmatch(source.name) and source.is_file():
            target = output / name / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
(output / "west-projects.json").write_text(json.dumps(manifest, indent=2) + "\n")
