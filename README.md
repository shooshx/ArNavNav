ArNavNav is a simple and yet powerful 2D navigation library for games

Current features
- Define polylines that make the circumference of the map and obstacles in it
- Restricted Delaunay triangluation of the enclosed area
  - handles multiple regions on mesh
- Position agents on the map and set goal positions
- A* planning on the navigation mesh using segment mid-points
- path planning using string-pulling algorithm
- Fast collision detection of multiple agents that move in the scene
- Web GUI using Qt QWebView or Emscripten

BUGs:
- strange corner added to plan

TBD
- extract polylines from terrain mesh
- handling self intersections
  - inside poly intersects outside

- UI
  - show path, future path
  - define speed
  - clear frames
  - clear scene
  - BUG: backtrack, play: not doing the same thing
  - single step
- remesh should do replan for all

Maybe
- more accurate A* - not between mid-segments
- connect different meshes to make floors?
- add agent radius consideration in A* (can't pass)

