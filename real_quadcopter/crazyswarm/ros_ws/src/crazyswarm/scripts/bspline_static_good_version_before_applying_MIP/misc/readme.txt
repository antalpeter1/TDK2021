1. run main.py. This will generate a yaml file that stores the initial positions. If the file exists already, this can be skipped.
2. run takeStartingPosition.py.
- randomly placed drones will fly to their assigned initial positions. No collision avoidance.
- they land. push a key on the keyboard to start the optimization.
- optimization done, look at the figure it produced.
- if okay, push a key on the keyboard
- drones will fly on the calculated path and back and then they land.


This optimization thing should be self-contained.
When we invoke it, we can feed-in obstacle coordinates optionally.
We can also feed in starting and final positions.
Once done, we will plot the environment.
Then do the optimization and plot the result.

