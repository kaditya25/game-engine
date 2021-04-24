# Tournament Instructions

## Goals
Vision Test: Estimate the red and blue balloon center locations to within 10
cm based on a set of images and their corresponding metadata.

Obstacle Course: Command a quadcopter to pop two balloons and return to the
start position as quickly as possible.

## Important Dates
- April 29: Pre-tournament starts. First day that students may submit
  binaries
- April 30: First pre-tournament evaluation date. Binaries and tournament standing
  are evaluated at 9 am. Binaries will be updated and evaluated every week day
  at 9 am thereafter until the end of the pre-tournament.
- May 12: Final day of pre-tournament. Final evaluation at 9 am. Final map of
  arena released (this one will be used on the day of the tournament).
- May 13: Tournament binaries (both obstacle course and vision test) must be submitted by 9 am.    
- May 14, Tournament day. 9 am

## Obstacle Course Notes

### Interface
Students must complete the function
`StudentAutonomyProtocol::UpdateTrajectories` located in the file
`game-engine/src/autonomy_protocol/student_autonomy_protocol.h`. This function
provides access to map data, balloon data, and the current state of the
quadcopter, and expects students to specify quadcopter trajectories.

Note that the interface only specifies the input and output (quad state ->
quad trajectory), but not how to accomplish it. It is up to students to
determine how best to complete the function.

While the entire project boils down to one function in one file, understand that
the problem set before you is not trivial. You will find that the problem of
prescribing a time-optimal trajectory through a cluttered environment in the
presence of disturbances is very difficult. 

For lab 4, students planned paths through a cluttered 2D environment. For that
lab, much of the 2D files were already written, tested, debugged, and vetted.
For this project, students are provided with little 3D support. It is
completely up to them to determine how they will tackle this problem.

Students are allowed to add any additional files, functions, or enhancements
to their Game Engine code base. However, students are not allowed to modify or
delete current functionality without the written permission of one of the TAs.

### Code Restrictions
1) Students must not alter any code in the src/exe folder
2) Students must not alter any of the AutonomyProtocol interfaces
3) Students must not change or remove any core functionalities of the GameEngine
without written permission of one of the TAs.

### Running the Game Engine
To run the final project, follow the build/run instructions in `README.md` in
the top-level `game-engine` directory. Student autonomy protocols are compiled
into an executable called `student_autonomy_protocol`. 

<!--- ### Tagging releases
During the pre-tournament and tournament, your team will want the TA to
evaluate only your *approved* `student_autonomy_protocol` binary, which may
not be the one in the master branch of your team's GitLab repository.  You'll
want to *tag* an approved release so that the TA knows which one to test.  The
TA will test whichever revision is tagged `release`.

Suppose you're happy with the way your autonomy protocol is running on your
local machine and you'd like to tag the current version as `release`.  Perform
the following steps.

First, commit your current version and push to `origin` on branch `master`:
```bash
git commit -am "WRITE A COMMIT MESSAGE"
git push origin master
```

Second, if you already have a tag named `release`, then delete this
tag locally and at `origin`:
```bash
cd ~/Workspace/game-engine
git tag -d release
git push origin --delete release
```

Third, tag your current commit as `release` and push the tag to `origin`.  In
the example below, the annotated tag is labeled with example message
"pre-tournament day 3."
```bash
git tag -a release -m "pre-tournament day 3"
git push origin release
```-->

## Tournament Binaries
To allow teams to evaluate their standing with respect to the other teams, but
prevent code-sharing, the pre-tournament will employ binaries. Each
game-engine repository is set up to build a binary called
"student_autonomy_protocol" and place it into a game-engine/bin folder. Teams
will be required to submit this binary for pre-tournament evaluation.  A TA
will pull these binaries every weekday at 9 am and evaluate tournament
standings.

Clone the `tournament-binaries` repository as follows:
```bash
cd ~/Workspace
git clone https://gitlab.com/todd.humphreys/tournament-binaries.git
```

### Binary Submission
To submit a binary, copy your `student_autonomy_protocol` binary into your
team's directory in `tournament-binaries` and then push to the remote
repository:

```bash
cd ~/Workspace/tournament-binaries/YOUR_TEAM_NAME
cp ../../game-engine/bin/student_autonomy_protocol .
git add student_autonomy_protocol
git push origin master
```

### Public Binaries
All submitted binaries become public. Thus, all teams may view their
competition's binaries.  To evaluate an opponent's strategy, launch all
tournament executables as normal, but substitute an opponent's
`student_autonomy_protocol`.


