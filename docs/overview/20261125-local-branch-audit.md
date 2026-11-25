---
title: "Local branch audit (strict 3-day window)"
created_at: 2026-11-25T00:00:00+00:00
status: audit complete; follow-up TODO recorded
---

# Local branch audit (strict 3-day window)

## Scope and method

Every local branch was inspected (`git for-each-ref refs/heads`, 62 branches
including the 37 `agent-*` worktrees). Per branch:

```bash
git log --since=3.days origin/master..<branch>
git cherry -v origin/master <branch>
git -C <worktree> status --porcelain
```

Only commits with a committer date inside the last three days (since
2026-11-22) are in scope, as requested. Older changesets are deliberately not
triaged. In `git cherry`, `-` means an equivalent patch is already in
`origin/master`; `+` means no equivalent patch is there.

No worktree has uncommitted changes, so the audit covers committed work only.

## All branches

### In scope (tip dated 2026-11-23 or 2026-11-24)

| Branch | Tip | Recent local commits | Upstream equivalent? |
| --- | --- | --- | --- |
| `backup/pre-pull-20261124` | `17b4c0f0` | 22 | all `-` |
| `local-changes` | `fdbe5de9` | 51 | all `-` |
| `local-fixes` | `04e87a9e` | 38 | 5 `+`, rest `-` |
| `merge-20261124` | `ee33aaf6` | 30+ | all `-` |
| `temp` | `ac454d94` | 30+ | all `-` |
| `master` | `a3be3549` | current pushes | pushed to `origin/master` |

The five recent `+` commits, all on `local-fixes`, are the only work in the
three-day window without an equivalent patch upstream:

- `04e87a9e` — `feat(scripts): build the MNNConvert CLI alongside the converter lib`
- `ea139531` — `fix(isp): mnn-gated cleanups — deferred init, drop i32 self-cast, fmt tuple`
- `a796111e` — `fix(android): correct runtime_libs path to one dir up from android-app`
- `72ad0292` — `docs: finish cam-app to cam-disp/android-app cleanup`
- `2b796345` — `fix(android): drop AppCompat theme and version-qualify adaptive icons for aapt2`

They are recorded as TODO P2f in `docs/overview/ROADMAP.md`; no cherry-pick was
attempted in this session because each needs a compatibility review against the
current tree (the app crate was renamed and the workspace layout changed).

### Out of scope (tip older than three days)

All 37 `agent-*` worktrees sit on `145d5638` (2026-09-21). Their tip patch is
already upstream-equivalent (`-`), and they have no commits in the window.

The remaining branches have no commits in the window and are therefore not
triaged, per the strict rule:

`android-app-makefile`, `android-hal-impl`, `feat/buffer-convert-v2`,
`feature/lf-fifo-onnx-zero-copy-microblocks`, `feature/update-spec`,
`feature/v3`, `fix/warning-doc-cleanup`, `legacy`, `main`, `master-20260806`,
`master-20260921`, `master-old`, `old`, `old-master`, `softisp-temp`, `staging`,
`temp-filter`, `temp-filter2`, `wip/android-camera-hal`.

## Force-push note

The old `master` history wiped at `ee13e72e` was also checked. Inside the
three-day window its patch-unique commits were `53deb251`, `ea015fb3`,
`1e68245e`, and `11ddd966`; all four were already restored by the rebase that
produced `9e9acaeb`. Older wiped patches are outside the requested window.
