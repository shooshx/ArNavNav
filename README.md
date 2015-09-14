ArNavNav is a simple and yet powerful 2D navigation library for games

Current features
- Define polylines that make the circumference of the map and obstacles in it
- Restricted Delaunay triangluation of the enclosed area
- Fast collision detection of multiple agents that move in the scene
- A* planning on the navigation mesh using segment mid-points

next fixes



TBD

- add agent radius consideration in A*
- reduce allocation and small objects
  - BihTree init
- extract polylines from terrain mesh
- more accurate A* - not between mid-segments
- better GUI for defining the scene
- connect different meshes to make floors?
- WebUI asm.js
- handling self intersections
