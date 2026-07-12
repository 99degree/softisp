#!/usr/bin/env python3
"""Download GitHub Actions CI logs and grep for errors.

Usage:
  python3 scripts/get_ci_logs.py [--run RUN_ID] [--grep PATTERN]
  
  If no RUN_ID specified, fetches latest run on the current branch.
  Default grep pattern: "error"
"""

import argparse
import getpass
import json
import os
import re
import sys
import tempfile
import zipfile
from urllib.request import Request, urlopen
from urllib.error import HTTPError

OWNER = "99degree"
REPO = "softisp"

def api_get(url, token):
    """Make authenticated GET request to GitHub API."""
    req = Request(url)
    req.add_header("Authorization", f"Bearer {token}")
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("X-GitHub-Api-Version", "2022-11-28")
    try:
        with urlopen(req) as resp:
            return json.loads(resp.read())
    except HTTPError as e:
        print(f"HTTP {e.code}: {e.reason}", file=sys.stderr)
        body = e.read().decode()
        if body:
            print(body[:500], file=sys.stderr)
        sys.exit(1)

def api_download(url, token, output_path):
    """Download a file from GitHub API."""
    req = Request(url)
    req.add_header("Authorization", f"Bearer {token}")
    req.add_header("Accept", "application/vnd.github+json")
    try:
        with urlopen(req) as resp:
            with open(output_path, "wb") as f:
                f.write(resp.read())
        return output_path
    except HTTPError as e:
        print(f"HTTP {e.code}: {e.reason}", file=sys.stderr)
        body = e.read().decode()
        if body:
            print(body[:500], file=sys.stderr)
        return None

def get_latest_run(token, branch="master"):
    """Get the latest workflow run on the given branch."""
    url = f"https://api.github.com/repos/{OWNER}/{REPO}/actions/runs?per_page=5&branch={branch}&status=completed"
    data = api_get(url, token)
    runs = data.get("workflow_runs", [])
    if not runs:
        print("No completed runs found.", file=sys.stderr)
        sys.exit(1)
    return runs[0]

def get_check_runs(token, run_id):
    """Get all check runs for a workflow run."""
    url = f"https://api.github.com/repos/{OWNER}/{REPO}/check-runs?filter=latest&per_page=20"
    # check runs are per-commit, not per-workflow-run
    # We need the commit SHA first
    run = api_get(f"https://api.github.com/repos/{OWNER}/{REPO}/actions/runs/{run_id}", token)
    head_sha = run["head_sha"]
    url = f"https://api.github.com/repos/{OWNER}/{REPO}/commits/{head_sha}/check-runs"
    return api_get(url, token), head_sha

def get_job_logs(token, run_id):
    """Get all job logs for a workflow run as a zip."""
    url = f"https://api.github.com/repos/{OWNER}/{REPO}/actions/runs/{run_id}/logs"
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".zip")
    path = api_download(url, token, tmp.name)
    if not path:
        print("Failed to download logs", file=sys.stderr)
        sys.exit(1)
    return path

def extract_and_grep(zip_path, grep_pattern):
    """Extract zip and grep all log files."""
    results = []
    extract_dir = tempfile.mkdtemp(prefix="ci_logs_")
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(extract_dir)
    
    pat = re.compile(grep_pattern, re.IGNORECASE)
    for root, dirs, files in os.walk(extract_dir):
        for fn in sorted(files):
            fpath = os.path.join(root, fn)
            try:
                with open(fpath, "r", errors="replace") as f:
                    lines = f.readlines()
            except Exception:
                continue
            for i, line in enumerate(lines, 1):
                if pat.search(line):
                    results.append((fn, i, line.rstrip()))
    return results, extract_dir

def main():
    parser = argparse.ArgumentParser(description="Download and grep CI logs")
    parser.add_argument("--run", type=int, help="Workflow run ID (default: latest)")
    parser.add_argument("--branch", default="master", help="Branch name (default: master)")
    parser.add_argument("--grep", default="error", help="Pattern to search (default: 'error')")
    parser.add_argument("--token", help="GitHub PAT (will prompt if not provided)")
    parser.add_argument("--save-zip", help="Save the zip file to this path")
    args = parser.parse_args()

    token = args.token or getpass.getpass("GitHub PAT: ")

    if args.run:
        run_id = args.run
    else:
        run = get_latest_run(token, args.branch)
        run_id = run["id"]
        print(f"Latest run: #{run_id} ({run['head_commit']['message'][:60]})")
        print(f"  Conclusion: {run['conclusion']}, SHA: {run['head_sha'][:8]}")
        print()

    # Get check runs
    check_data, head_sha = get_check_runs(token, run_id)
    print(f"Check runs for commit {head_sha[:8]}:")
    for cr in check_data.get("check_runs", []):
        print(f"  {cr['name']}: {cr['conclusion']}")
    print()

    # Download logs
    print("Downloading logs (this may take a while)...")
    zip_path = get_job_logs(token, run_id)
    
    if args.save_zip:
        import shutil
        shutil.copy(zip_path, args.save_zip)
        print(f"Logs saved to {args.save_zip}")

    # Grep
    print(f"\nSearching for '{args.grep}' in logs...")
    results, extract_dir = extract_and_grep(zip_path, args.grep)
    
    if not results:
        print("No matches found.")
    else:
        # Group by file
        current_file = None
        for fn, line_no, line in results:
            if fn != current_file:
                print(f"\n--- {fn} ---")
                current_file = fn
            print(f"  L{line_no}: {line}")
    
    print(f"\nTotal matches: {len(results)}")
    print(f"Logs extracted to: {extract_dir}")
    
    # Cleanup zip
    os.unlink(zip_path)

if __name__ == "__main__":
    main()
