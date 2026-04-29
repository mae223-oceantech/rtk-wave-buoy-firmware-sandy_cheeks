# Getting Updates from the Main Repo

When your instructor pushes updates to the main repository, you can pull those changes into your own GitHub Classroom repo without losing any of your own work. This works even if you have already been editing files.

---

## How It Works

Your GitHub Classroom repo and the instructor's main repo are separate. To receive updates, you connect them with a second remote called `upstream`. You then fetch and merge updates from `upstream` into your own repo — the same way git merges any two branches.

- If the instructor changed files you haven't touched → merges automatically, nothing to do
- If the instructor changed the same file you edited → git flags a conflict, you resolve it manually

---

## Step 1: One-Time Setup (do this once after accepting the assignment)

```bash
git remote add upstream https://github.com/mae223-oceantech/mae223-ocean-tech-2026-rtk-wave-buoy-firmware-mae223-ocean-tech-2026-rtk-wave-buoy-firmware-rtk-wave
```

Verify it worked:

```bash
git remote -v
```

You should see:

```
origin    https://github.com/YOUR-USERNAME/YOUR-CLASSROOM-REPO (fetch)
origin    https://github.com/YOUR-USERNAME/YOUR-CLASSROOM-REPO (push)
upstream  https://github.com/mae223-oceantech/mae223-ocean-tech-2026-rtk-wave-buoy-firmware-mae223-ocean-tech-2026-rtk-wave-buoy-firmware-rtk-wave (fetch)
upstream  https://github.com/mae223-oceantech/mae223-ocean-tech-2026-rtk-wave-buoy-firmware-mae223-ocean-tech-2026-rtk-wave-buoy-firmware-rtk-wave (push)
```

---

## Step 2: Commit Your Work First (every time before merging)

**This is required.** Git will refuse to merge if you have unsaved (uncommitted) changes. Before pulling any update, commit whatever you have been working on — even if it is not finished:

```bash
git add .
git commit -m "WIP: save work before merging instructor update"
```

---

## Step 3: Fetch and Merge the Update

```bash
git fetch upstream
git merge upstream/main
```

- `git fetch upstream` — downloads the instructor's latest changes without touching your files
- `git merge upstream/main` — merges those changes into your branch

If there are no conflicts, git merges automatically and you are done. Push to your repo to save:

```bash
git push origin main
```

---

## Resolving Merge Conflicts

If you and the instructor both changed the same file, git will flag a conflict. The affected file will contain markers like this:

```
<<<<<<< HEAD
your version of the line
=======
instructor's version of the line
>>>>>>> upstream/main
```

To resolve:
1. Open the file and edit it — keep your version, the instructor's version, or combine both
2. Remove all the `<<<<<<<`, `=======`, and `>>>>>>>` markers
3. Save the file
4. Stage and commit:

```bash
git add <filename>
git commit -m "Resolve merge conflict with upstream update"
git push origin main
```

---

## Common Mistakes

| Mistake | What happens | Fix |
|---------|-------------|-----|
| Running `git merge` without committing first | Git refuses with "you have unstaged changes" | Run `git add .` and `git commit` first |
| Running `git pull` instead of `git fetch` + `git merge` | Pulls from your own repo, not the instructor's | Use `git fetch upstream` then `git merge upstream/main` |
| Skipping `git remote add upstream` | "upstream" not found error | Run the one-time setup command first |
| Trying to push to the instructor's repo | Permission denied | You can only push to `origin` (your own repo) |

---

## Summary

```bash
# One-time setup
git remote add upstream https://github.com/mae223-oceantech/mae223-ocean-tech-2026-rtk-wave-buoy-firmware-mae223-ocean-tech-2026-rtk-wave-buoy-firmware-rtk-wave

# Every time the instructor announces an update
git add .
git commit -m "Save work before merging update"
git fetch upstream
git merge upstream/main
git push origin main
```
