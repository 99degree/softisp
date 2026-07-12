---

## 3.5. GIT WORKFLOW: COMMIT → PULL/REBASE → PUSH

**MANDATORY WORKFLOW FOR EVERY COMMIT**

```bash
# 1. Stage and commit your changes
# git add <files>
# git commit -m "descriptive message"

# 2. Pull latest changes with rebase (NOT merge)
# git pull --rebase origin main

# 3. Resolve any conflicts if they arise
# git status
# # fix conflicts if any
# git add <resolved-files>
# git rebase --continue

# 4. Push to remote
# git push origin main
```

**RULES**:

* **ALWAYS rebase, never merge** - keeps history linear and clean
* **Commit before pulling** - ensures your changes are on top of the latest upstream
* **Resolve conflicts immediately** - don't let them accumulate
* **Push immediately after successful rebase** - don't let local commits pile up
* **Never force push to shared branches** (`main`, `master`) without explicit coordination

**ANTI-PATTERNS TO AVOID**:
* ❌ `git pull` (creates merge commits)
* ❌ `git push --force` on shared branches
* ❌ Accumulating multiple commits before pushing
* ❌ Committing broken code with "will fix later"

---

## 3.6. REPOSITORY BOUNDARY ENFORCEMENT

**CRITICAL: NEVER ADD FILES OUTSIDE THE REPOSITORY ROOT**

* **NEVER** stage or commit files with absolute paths outside the repo (e.g., `~/android-sdk/`, `/home/user/...`, `/opt/...`, `C:\Users\...`)
* **NEVER** use `git add ~/` or `git add /absolute/path` 
* **ALWAYS** verify staged files with `git status` before committing
* **ALWAYS** use relative paths from repo root for all git operations
* If external dependencies are needed, document them in README or setup scripts — **do not commit them**

**IF VIOLATION OCCURS**:
1. Immediately `git rm -r --cached <outside-path>` to unstage
2. Add path to `.gitignore`
3. If already pushed: `git filter-repo --path <outside-path> --invert-paths --force` (requires fresh clone)
4. Force push cleaned history

**VERIFICATION COMMAND** (run before every commit):
```bash
git status | grep -E "^(\s+)?(new|modified|deleted):.*[~/]" && echo "❌ EXTERNAL FILES DETECTED" || echo "✅ Clean"
```