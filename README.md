ArNavNav is a simple and yet powerful 2D navigation library for games

Current features
- Define polylines that make the circumference of the map and obstacles in it
- Restricted Delaunay triangluation of the enclosed area
- Fast collision detection of multiple agents that move in the scene
- A* planning on the navigation mesh using segment mid-points
- path planning using string-pulling algorithm
- Web GUI using Qt QWebView or Emscripten

TBD
- extract polylines from terrain mesh
- handling self intersections
  - inside poly intersects outside
- multiple regions on mesh
- UI
  - set agent radius
  - show triangulation, polylines
  - show path, future path

Maybe
- more accurate A* - not between mid-segments
- connect different meshes to make floors?
- add agent radius consideration in A* (can't pass)

