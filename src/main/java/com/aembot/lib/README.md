# AEMLib
AEMLib is team 6443 AEMBOT's bespoke library for FRC. It contains code to be reused every season.

AEMLib is split off into its own repo, and included in other repos as a subtree. If you wish to
develop for or use AEMLib, check out the AEMTemplate repo or our current season repo.

To push or pull changes to/from the upstream AEMLib repo, use these commands:

First off, you'll need to add the AEMLib upsteam as a remote:
```bash
git remote add aemlib https://github.com/AEMBOT/AEMLib.git
```

To push commits made on your repo up to AEMLib upstream:
```bash
git subtree push --prefix=src/main/java/com/aembot/lib aemlib <insert branch name here. ie. "2027-changes">
```

To pull commits from the AEMLib upstream to your repo (generally, you'll only want to do this on AEMTemplate.):
```bash
git subtree pull --prefix=src/main/java/com/aembot/lib aemlib main
```
