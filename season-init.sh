#!/usr/bin/env bash

echo "This script will set up this local copy of the template repo (which MUST be the working directory) to become a new season repo."

read -p "Are you sure you want to do this? [y/N] " -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    read -p "Paste the link to the empty github repo for the new season. (Ie. 'https://github.com/AEMBOT/FRC_2026.git'): " -r repo_url
    
    git remote rename origin template
    git remote add origin "$repo_url"
    git remote add lib https://github.com/AEMBOT/AEMLib.git

    # This feels wrong, but it can still finish executing
    git rm ./season-init.sh
    git commit -m "Season-specific git init"

    echo "Season repo set up. Run 'git push -u origin main' to push to the new github repo. This script will now self destruct :3"

else
    echo "Cancelled."
fi
