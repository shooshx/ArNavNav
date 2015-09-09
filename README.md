ArNavNav is a simple and yet powerful 2D navigation library for games

Current features
- Define polylines that make the circumference of the map and obstacles in it
- Fast collision detection of multiple agents that move in the scene
- A* planning on the navigation mesh

next fixes
- BUG - achieving goal and then backing up due to collision
- goal point should not be projection, should be as tight as possible
- midPnt is changed in the mesh - bad for reusing the mesh

TBD
- finalize base features
- add agent radius consideration in A*
- reduce allocation and small objects
- extract polylines from terrain mesh
- more accurate A* - not between mid-segments
- better GUI for defining the scene
- connect different meshes to make floors?

