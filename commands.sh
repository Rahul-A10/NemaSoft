# Remove from index, add to .gitignore and commit
git rm --cached "x64/debug/opencv_world4110d.dll"
echo "x64/debug/" >> .gitignore
git add .gitignore
git commit -m "Remove OpenCV DLL and ignore build outputs"

# Use BFG to delete the file from history (requires Java & BFG)
bfg --delete-files opencv_world4110d.dll

# Final cleanup and force push
git reflog expire --expire=now --all
git gc --prune=now --aggressive
git push origin main --force