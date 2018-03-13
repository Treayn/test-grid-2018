# test-grid-2018

**This is a piece of code written for a n--a--s--a competition (Still in progress, making sure this isn't tagged as such so other teams don't find it.)**

## Overview

The competition is essentially a scavenger hunt on Mars. 3 Rovers must gather as many soil samples/scientific artifacts as possible in 30 minutes.

Our solution was to create a 150-by-150 virtual grid representing the 15-by-15 meter space. That results in 100 cm/grid cell.

### Heuristics

As the rovers drive through the map, they broadcast their positions to other rovers, which gets marked on the grid.
This is used as a heuristic for pathfinding algorithms like Djikstra's/A*.

The rovers also broadcast the location of samples/artifacts as they're discovered.
This allows the rovers to decide where to go after they've completed a prior task.

### Dynamic Allocation

In the final rounds of the competition, we have 5 Rovers, a 22-by-22 grid, and an hour.
There is no way to tell if the current match is a qualifier or a final.
Our solution is to use a grid (represented by a 2D deque) that can dynamically expand when the rovers pass through the bounds.
