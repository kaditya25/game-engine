# Tournament Instructions

## Goals
Vision Test: Estimate the red and blue balloon center locations to within 10
cm based on a set of images and their corresponding metadata.

Obstacle Course: Command a quadcopter to pop two balloons and return to the
start position as quickly as possible.

## Important Dates
- April 29: Pre-tournament starts. First day that your team may submit
  `student_autonomy_protocol` binaries
- April 30: First pre-tournament evaluation date. Code tagged 'release' is
  evaluated at 9 am. 'release' code will be pulled and evaluated every week day
  at 9 am thereafter until the end of the pre-tournament.
- May 12: Final pre-tournament evaluation at 9 am. Final map of
  arena released: this is the map that will be used on the day of the tournament.
- May 13: Tournament code for both obstacle course and vision test must be tagged 'release' by 9 am.    
- May 14, Tournament day. 9 am to 12 pm.

## Obstacle Course Notes

### Interface
Your team must complete the function
`StudentAutonomyProtocol::UpdateTrajectories` located in the file
`game-engine/src/autonomy_protocol/student_autonomy_protocol.h`. This function
provides access to map data, balloon data, and the current state of the
quadcopter.  It returns a specified quadcopter trajectory.

Note that the interface only specifies the input and output (quad state ->
quad trajectory), but not how to accomplish the implied task. It is up to you
to determine how best to complete the function.

The entire obstacle course challenge thus boils down to writing a single
function in one file.  But understand that the problem set before you is not
trivial. The problem of prescribing a time-optimal trajectory through a
cluttered environment in the presence of disturbances is very difficult.

For lab 4, you planned paths through a cluttered 2D environment. For that lab,
much of the 2D files were already written, tested, debugged, and vetted.  For
this project, you are provided with little 3D support. It is completely up to
you to determine how you will tackle this problem.

You are allowed to add any additional files, functions, or enhancements to
your `game-engine` code base. However, you are not allowed to modify or delete
current functionality without the written permission of one of the TAs.

### Code Restrictions
1) Don't alter any code in the src/exe folder
2) Don't  alter any of the `AutonomyProtocol` interfaces
3) Don't change or remove any core functionalities of `game-engine`
without written permission of one of the TAs.

### Running the Game Engine
To run `game-engine`, follow the build/run instructions in `README.md` in the
top-level `game-engine` directory. Your autonomy protocol will be compiled
into an executable called `student_autonomy_protocol`.

### Tagging releases for Evaluation
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
```

A TA will pull your `release` tag every evaluation day at 9 am and evaluate
your pre-tournament standing.


## Availability of Binaries

To allow teams to evaluate their standing with respect to the other teams, but
prevent code-sharing, the TAs will post all teams' `student_autonomy_protocol`
binaries.  To evaluate an opponent's strategy, launch all tournament
executables as normal, but substitute an opponent's
`student_autonomy_protocol` for yours.  Note that if your file paths are
different from the ones other students are using, dynamic linking of libraries
may fail.


