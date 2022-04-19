# Tournament Setup

This document provides instructions on how to set up for the Aerial Robotics
course pre-tournament and tournament. Please read this document carefully. Use
your best judgement when following the instructions, adapting where necessary
for your machine and setup.

## GitLab account
Each team member should already have created a GitLab account.  If not, then create one.

## Create a remote GitLab repository
Designate one of your team members to host your team's remote repository. This
person should create a `game-engine` repository on his or her account in
GitLab. Name both the repository and the slug `game-engine`. Make the
repository private to ensure that only you may view this code. You don't want
your opponents to see your secrets!

In the repository settings/members, add your team members as Maintainer to
your repository. This allows them to view and edit the code. Also add our
superlative TAs Robert and Zacharias as Reporter so they can follow your
progress. Their GitLab handles are @roberttenny and @zackkomo.

## Clone `game-engine`
In the `~/Workspace` directory, either clone a fresh copy of the
`game-engine` repository (if you want to start with a clean slate), or
update your existing local `game-engine` repository (if one already exists at
that location and you want to keep it).

### To clone a fresh copy:
```bash
cd ~/Workspace/
git clone https://gitlab.com/radionavlab/public/game-engine.git
cd game-engine
git submodule update --init --recursive
```

### To update an existing copy
The code below assumes `origin` refers to
`https://gitlab.com/radionavlab/public/game-engine.git`.

Pull the latest changes from `origin` into your code:
```bash
cd ~/Workspace/game-engine
git pull origin master
# resolve merge conflicts as necessary
git submodule update --init --recursive
```

## Configure the repository
Configure your `game-engine` repository with your name and email. All commits
that you make will be signed with this information.

```bash
cd ~/Workspace/game-engine
git config user.name "YOUR NAME HERE"
git config user.email "YOUR EMAIL HERE"
```

Your team's work will involve three repositories: (1) the `game-engine`
repository on your local machine, (2) your team's remote `game-engine`
repository on GitLab, and (3) the original `game-engine` repository on
GitLab. The three of these interact as follows: TAs will periodically push
patches and changes to the original `game-engine` repository. You will
pull down and merge these changes into your local repository.  Likewise, you
will pull down and merge changes your teammates have made from your team's
remote repository.  Finally, you will push changes present in your local
repository to your team's remote repository so they are safely stored and
accessible to your teammates.

These three repositories will be referred to as
- Your local `game-engine` repository -> `local`
- Your team's remote repository on GitLab -> `origin`
- The original `game-engine` remote repository on GitLab -> `source`

Configure these repositories in the git settings as follows. 
```
cd ~/Workspace/game-engine
git remote rename origin source
git remote add origin https://gitlab.com/YOUR_GITLAB_USERNAME/game-engine.git
# For example:
#  git remote add origin https://gitlab.com/tony.stark/game-engine.git
```

Now that your local repository is configured, push your local contents to
`origin`, your team's remote GitLab repository:
```bash
cd ~/Workspace/game-engine
git push -u origin --all
git push -u origin --tags
```

After pushing, you should see that your `game-engine` repository on GitLab has
the same contents as your local repository.  Remember, only one member of your
team should create your team's remote repository and make this initial push;
otherwise, you'll end up creating multiple remote repositories for your team.

## Pushing changes

To push changes from `local` to `origin` so that your teammates may pull those
changes, first, add any new files you created:
```bash
git add LIST_OF_NEW_FILES
```

Next, commit these files and any previously-added files that have been
modified to your local repository, adding a commit message:
```bash
git commit -am "THIS IS WHERE YOUR COMMIT MESSAGE GOES. IT SHOULD DESCRIBE WHAT YOU CHANGED."
```

Finally, push to `origin` on branch `master`:
```bash
git push origin master
```

## Pulling changes
Suppose a change has been made to a remote repository (e.g., your teammate has
pushed to `origin` or a TA has changed `source`), and suppose you want to pull
in and merge these changes with `local`. First commit your local changes as
described above.  Next, pull from the remote repository:

To pull from `source`:
```bash
git pull source master
```

To pull from `origin`:
```bash
git pull origin master
```

## Create a remote GitLab repository for the Vision Test
Follow steps similar to the above to create your team's remote repository for
the tournament's Vision Test.  Name the remote repository `balloon-locator`.
Rename your local `lab5-aerial-robotics` repository `balloon-locator`.

As before,
- Add your team members as Maintainer and the TAs as Reporter to the remote repository.
- Rename `origin` as `source` and add your new remote repository as `origin`.
- Push your local repository to `origin`.
- Configure `balloon-locator` with your name and email.

## Accessing Other Teams' Binary Executables
Clone the `tournament-binaries` repository as follows:
```bash
cd ~/Workspace
git clone https://gitlab.com/todd.humphreys/tournament-binaries.git
```
