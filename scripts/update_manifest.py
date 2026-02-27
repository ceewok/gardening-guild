import hashlib
import json
import subprocess
from pathlib import Path

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        # Read entire file and normalize CRLF to LF
        content = f.read().replace(b"\r\n", b"\n")
        h.update(content)
    return h.hexdigest().upper()

def main():
    root = Path(__file__).resolve().parent.parent
    manifest_path = root / "scripts" / "manifest.json"
    
    if not manifest_path.exists():
        print("manifest.json not found!")
        return
        
    with manifest_path.open("r") as f:
        manifest = json.load(f)
        
    updated = False
    for entry in manifest.get("files", []):
        rel_path = entry.get("path")
        if not rel_path: continue
        
        file_path = root / rel_path
        if file_path.exists():
            new_hash = sha256_file(file_path)
            if entry.get("sha256") != new_hash:
                entry["sha256"] = new_hash
                updated = True
                print(f"[ERC2] Updated hash for {rel_path}")
        else:
            print(f"[ERC2] Warning: file {rel_path} not found.")
            
    if updated:
        with manifest_path.open("w", newline="\n") as f:
            json.dump(manifest, f, indent=4)
            f.write("\n")
        
        # Add the manifest changes back to the git staging area so it gets committed
        subprocess.run(["git", "add", "scripts/manifest.json"], cwd=root)
        print("[ERC2] manifest.json updated and staged.")

if __name__ == "__main__":
    main()
