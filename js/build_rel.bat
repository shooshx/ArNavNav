em++ -O2 -std=c++11 --memory-init-file 0 js_main.cpp ../Agent.cpp ../poly2tri/adapter.cc ../poly2tri/advancing_front.cc ../poly2tri/shapes.cc ../poly2tri/sweep.cc ../poly2tri/sweep_context.cc ../BihTree.cpp ../Document.cpp ../Mesh.cpp -o js_main.html -s EXPORTED_FUNCTIONS="['_cpp_start', '_added_poly_point', '_moved_object', '_started_new_poly', '_added_agent', '_add_goal', '_remove_goal', '_set_goal', '_cpp_progress']"