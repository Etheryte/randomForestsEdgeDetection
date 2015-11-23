


Make graph of clusters where edges mark shared edge parts, then join ones that are above threshold

Make graphs for identifying goals: horizontalish groups and their distances, vertical groups that are between the former and above the center 2/3?,
only if all of them exist may it be a goal + other things like ratio check?

Edges: A lot of groups or just by mass that share a similar average direction. Keep only those that are horizontalish?:
we will lose close to field edge detection. Alternatively: if a lot of horizontal and only single area (so not an edge).

Ball: See what characteristics are present after merging clusters. Possibly a lot of small clusters in an area, must be mostly surrounded by green.


TODO: Average direction calculation, mod 180deg, when near the left half, change to right and then later switch back.
Only when largest delta under ~135 deg?

TODO: Can we use curvature after clusters are joined?



Finish implementing edge boxes?

Find joint points / crossovers: how to find?

Label the edges (edge, goal, ball etc.)
