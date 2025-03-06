# SDP
Everyone has their own branch. 
You only work in your own branch.
Main only gets updated once a week and requires approval.
To collaborate in between, merge with your collaborator's branch back and forth.
To change branches: git checkout main, git checkout origin/YourBranch, etc.
To merge changes your collaborator made: git merge origin/CollaboratorBranch, etc.
  -Then resolve any merge conflicts, sync changes.

To get yourself up to date with main:
  -git checkout main
  -git pull
  -git checkout origin/YourBranch
  -git merge main



To merge person A to person B
-person A has updates / new code

In old/out_of_date branch

git merge the_branch_with_updates
