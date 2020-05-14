# Final Project Instructions
## Goal
The goal is simple: command a quadcopter to find and pop two balloons and
return to the start position as quickly as possible.

## Interface
Students must complete the function `StudentAutonomyProtocol::UpdateTrajectories`
located in the file `src/autonomy_protocol/student_autonomy_protocol.h`. This
function provides access to map data, balloon data, and the current state of the
quadcopter and requires students to specify trajectories for the quadcopters. 

Note that the interface only specifies the input and output (quad state ->
quad trajectory), but not how to accomplish it. It is up to students'
discretion how to complete the function.

While the entire project boils down to one function in one file, understand that
the problem set before you is not trivial. You will find that the problem of
prescribing a time-optimal trajectory through a cluttered environment in the
presence of disturbances is very difficult. 

## Notes
For lab 4, students planned paths through a cluttered 2D environment. For that
lab, much of the 2D files were already written, tested, debugged, and vetted.
For this project, students are provided with little 3D support. It is completely
up to them to determine how they will tackle this problem.

Students are allowed to add any additional files, functions, or enhancements to
their Game Engine code base. However, students are not allowed to modify or
delete current functionality without the explicit permission of one of the TAs.

## Code Restrictions
1) Students must not alter any code in the src/exe folder
2) Students must not alter any of the AutonomyProtocol interfaces
3) Students must not change or remove any core functionalities of the GameEngine
without explicit permission of one of the TAs.

## Running the final project
To run the final project, follow the build/run instructions in `README.md` in
the top-level `game-engine` directory. Student autonomy protocols are compiled
into an executable called `student_autonomy_protocol`. Instead of running
`example_autonomy_protocol`, run `student_autonomy_protocol`.

## Tagging releases
During the pre-tournament and tournament, you'll want the TA to evaluate only
your *approved* `student_autonomy_protocol` binary, which may not be the one
in the master branch of your team's gitlab repository.  You'll want to *tag*
an approved release so that the TA knows which one to test.  The TA will test
whichever revision is tagged as `release`.

Suppose you're happy with the way your autonomy protocol is running on your
local machine and you'd like to tag the current version as `release`.  Then
perform the following steps.

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

