# Contribution — SoftISP

## Usual Practice

- **Inspect before editing.** Read the target file and its surrounding modules completely before making changes. Never assume file structure, layout, or line counts.
- **Small, atomic changesets.** Break large tasks into small, logical, incremental modifications. Do not bundle unrelated features or fixes into one massive commit.
- **Commit before pulling.** Always stage and commit your changes, then use `git pull --rebase origin master && git push origin master` as a one-liner. Never force-push to shared branches (`main`, `master`) without explicit coordination.
- **Verify before pushing.** Run the full CI gate locally (`cargo fmt --all -- --check`, clippy per-crate with `-D warnings`, and the workspace test suite) before pushing.
- **No workarounds.** Do not delete failing tests, swallow errors, or comment out problematic code to force a build to pass. Fix the root flaw.
- **No external files.** Never stage or commit files outside the repository root. Use relative paths from repo root for all git operations.
- **Branch off `master`.** `master` is the default branch; `origin/HEAD -> origin/master`.

## License & Re-licensing Rights

The SoftISP repository and all of its contents (source code, documentation, models, build scripts, and related materials) are authored and maintained by **99degree**.

**99degree retains the right to relicense the project to whatever license they choose**, at any time, including but not limited to changing between GPL-3.0-or-later, MIT, Apache-2.0, proprietary, or any other license. Contributions made to this project are accepted with the understanding that the project maintainer (99degree) may relicense the entire codebase, including contributed code, under different terms.

Existing files carry the license stated in their headers / `LICENSE`:
- **Code:** GPL-3.0-or-later (see [`LICENSE`](LICENSE)) unless a file header states otherwise.
- **Models:** CC BY-NC 4.0 (non-commercial) — see [`isp-rectifier/README.md`](isp-rectifier/README.md) for model-specific attribution requirements.

## Submitting Contributions

1. Fork the repo and create a feature branch from `master`.
2. Make your change, add tests, and run the CI gate locally.
3. Commit with a descriptive message focusing on *why* the change was made.
4. Rebase onto the latest `master` and push using the one-liner:
   ```bash
   git pull --rebase origin master && git push origin master
   ```
5. Open a pull request against `master`.

## Contact

Questions about licensing or relicensing: contact the repository owner, **99degree**.
