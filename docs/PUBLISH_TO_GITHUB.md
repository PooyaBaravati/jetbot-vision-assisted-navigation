# Publish to GitHub

Repository name suggestion: `jetbot-vision-assisted-navigation` (under `PooyaBaravati`).

1. Review [Before publishing](BEFORE_PUBLISHING.md), including rights to make the code public. You can start as a **private** repository and change visibility later.
2. Create an **empty** repository on GitHub: no auto-created README, `.gitignore`, or license, since this project already includes a README and `.gitignore`.
3. Transfer/unzip this repository folder to a machine with `git` and GitHub authentication. Run the following **inside** the unpacked project root:

```bash
git init
git add .
git status --short
git commit -m "Add JetBot CL camera-assisted navigation source and documentation"
git branch -M main
git remote add origin https://github.com/PooyaBaravati/jetbot-vision-assisted-navigation.git
git push -u origin main
```

Use a GitHub personal access token or authorized credential manager if requested; **never** place a token in repository files or paste one into chat. If the GitHub repository already has commits, don't force-push; merge/reconcile them first. If `git remote add origin` says it already exists, check `git remote -v` before changing anything.

The version uploaded here is **not** itself a GitHub repository yet. No upload/push was performed automatically.
