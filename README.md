ArNavNav is a simple and yet powerful 2D navigation library for games
Live demo at: http://shooshx.github.io/ArNavNav

Current features
- Define polylines that make the circumference of the map and obstacles in it
- Restricted Delaunay triangluation of the enclosed area
  - handles multiple regions on mesh
- Position agents on the map and set goal positions
- A* planning on the navigation mesh using segment mid-points
- path planning using string-pulling algorithm
- Fast collision detection of multiple agents that move in the scene
- Web GUI using Qt QWebView or Emscripten

BUG
- attack formation does not allow passing of other agents through
- don't replan too many times - replan in circle

TBD
- replan when stuck
   avoid when
   - tryed a few times from the same place (many agents attack small target)



- string-pull on radius-specific mesh
- narrow passage - with chopped VO - not dependent on neighbor dist
- UI
  - show path, future path
  - clear scene
  - BUG? backtrack, play: not doing the same thing
  - single step
  - rewind button
  - performance - read all movements

  - url
- don't mutate mesh in Astar  
  
- remesh should do replan for all ?


Maybe
- more accurate A* - not between mid-segments
- connect different meshes to make floors?
- add agent radius consideration in A* (can't pass)

