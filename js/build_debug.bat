em++ -g3 -O0 -std=c++11 -s ASSERTIONS=1 -s SAFE_HEAP=1 -s DEMANGLE_SUPPORT=1 --memory-init-file 0 js_main.cpp ../Agent.cpp ../poly2tri/adapter.cc ../poly2tri/advancing_front.cc ../poly2tri/shapes.cc ../poly2tri/sweep.cc ../poly2tri/sweep_context.cc ../BihTree.cpp ../Document.cpp ../Mesh.cpp -o js_main.html -s EXPORTED_FUNCTIONS="['_cpp_start', '_added_poly_point', '_moved_object', '_started_new_poly', '_added_agent', '_add_goal', '_remove_goal', '_set_goal', '_cpp_progress', '_serialize', '_deserialize', '_go_to_frame', '_change_size', '_update_goal']"