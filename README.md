## Arm Control

A common task in robot manipulation is to move the end-effector of a manipulator in a specified path (for example, to weld or paint a part). The code in the package mir rockin_control_fbm was developed for a competition 3 years ago; the task was to either follow (with the end-effector) a straight line of a given slope, or a sine wave path with a given frequency and amplitude.

### Problem

The code does not work (at least in simulation). It is also not easily transferable to other (similar tasks). For example, a new task is to follow a path which is not pre-specified, but instead on which is drawn on a sheet of paper. Hence the task would be to perceive and follow the path simultaneously. The existing code works only if the path is fully known beforehand.

### Proposed solution
* The non-working code has been narrowed down to mcr trajectory time parameterizer. Debug and make the component run.
* Refactor the code such that arm trajectories can be generated and executed as new information (about the path) is available.
