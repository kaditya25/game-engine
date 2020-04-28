# Final Project Setup
This document provides instructions on how to set up the final project for the
Aerial Robotics course. Please read the following document very carefully. Use
your best judgement when following this document. Do not just copy/paste, as
some commands may not be exactly applicable to your machine and setup.

## Create a gitlab account
Each team member must create a gitlab account.

## Create a remote gitlab repository
Each team should designate one team member to host their team's code. This step
must only be followed by that one team member.

One team member must create a 'game-engine' repository on gitlab. Name both the
repository and the slug as 'game-engine'. Make the repository private --- this
ensures that only you may view this code. Don't want your opponents to see your
secrets! 

In the repository settings/members, add your team members as maintainers to your
repository. This allows them to view and edit the code. Also add Dan and Nick
as Reporter to the code. We need the ability to audit your code. Our handles
are: @dmlachap @nickMont

## Clone 'game-engine'
In the '~/Libraries' directory, either clone a fresh copy of the
'game-engine-student' repository (if you want to start with a clean slate), or
update your existing local 'game-engine' repository (if one already exists at
that location and you want to keep it).  Name your local repository
'game-engine'.  To clone a fresh copy:
```bash
cd ~/Libraries/
git clone https://gitlab.com/todd.humphreys/game-engine-student.git game-engine
cd game-engine
git submodule update --init --recursive
```
To update an existing copy (assuming origin refers to
'https://gitlab.com/todd.humphreys/game-engine-student.git')
```bash
cd ~/Libraries/game-engine
git pull origin master
git submodule update --init --recursive
```

## Configure the repository
Configure your 'game-engine' repository with your name and email. By configuring
it, all commits that you make will be signed with this information.
```bash
cd ~/Libraries/game-engine
git config user.name "YOUR NAME HERE"
git config user.email "YOUR EMAIL HERE"
```

Three git repositories will be important to you: (1) the 'game-engine'
repository on your local machine, (2) your team's remote 'game-engine'
repository on gitlab, and (3) the original 'game-engine-student' repository on
gitlab. The three of these interact as follows: Dan and Nick will periodically
push patches and changes to the original 'game-engine-student' repository. You
will pull down and merge these changes into your local repository.  Likewise,
you will pull down and merge changes your teammates have made to your team's
remote repository.  Finally, you will push changes in your local repository to
your team's remote repository so they are safely stored and accessible to your
teammates.

These three repositories will be referred to as
- Your local 'game-engine' repository -> local
- Your team's repository on gitlab -> origin
- Original 'game-engine-student' repository on gitlab -> source

Configure these in the git settings as follows. Note that the lines begining
with '#' below are comments.
```
cd ~/Libraries/
git remote rename origin source
git remote add origin https://gitlab.com/YOUR_GITLAB_USERNAME/YOUR_REPOSITORY_NAME.git
# For example:
#   git remote add origin https://gitlab.com/tony.stark/game-engine.git
```

Now that your repository is configured, push your local copy to gitlab:
```bash
cd ~/Libraries/game-engine
git push -u origin --all
git push -u origin --tags
```
After pushing, you should see the repository on gitlab.  Remember, only one
member of your team should complete these steps; otherwise, you'll create
multiple remote repositories for your team.

## Pushing changes
You may want to push changes from 'local' to 'origin' so that your teammates
may pull those changes.

First, add any new files you created:
```bash
git add LIST_OF_NEW_FILES
```

Next, commit these files and add a commit message:
```bash
git commit -am "THIS IS WHERE YOUR COMMIT MESSAGE GOES. IT SHOULD DESCRIBE WHAT
YOU CHANGED"
```

Finally, push to origin:
```bash
git push origin master
```

## Pulling changes
Any time a change is made to a remote repository (e.g., your teammate has
pushed to 'origin' or Dan has changed 'source'), you will want to pull in and
merge these changes. The steps below assume that you have already committed
any changes in your local repository.

If the remote source (instructor code) is changed, pull from 'source':
```bash
git pull source master
```

If the remote origin (your team's code) is changed, pull from 'origin':
```bash
git pull origin master
```



