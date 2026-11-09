#!/usr/bin/env python3
# Final commit and push - self-deleting
import subprocess, os

os.chdir("/data/data/com.termux/files/home/softisp/cam-rust")

result = subprocess.run(["git", "add", "-A"], capture_output=True, text=True)
print("Add:", result.returncode)

result = subprocess.run(["git", "commit", "-m", "cleanup: remove gen_simple_onnx.py temp file"], capture_output=True, text=True)
print("Commit:", result.returncode, result.stdout)

result = subprocess.run(["git", "push", "--force", "origin", "main"], capture_output=True, text=True)
print("Push:", result.returncode, result.stdout, result.stderr)

os.remove(__file__)