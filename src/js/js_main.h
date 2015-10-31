#pragma once

#ifdef EMSCRIPTEN
typedef int ptr_t;
#else // Qt WebView
typedef long long ptr_t;
#define PTR_T long long // can't be a typedef, qt compiler doesn't like it
#endif

extern "C" {

void cpp_start();

void started_new_poly();
void added_poly_point(int x, int y);
void added_agent(int x, int y, float radius, float speed);
void moved_object(ptr_t ptr, int x, int y);

ptr_t add_goal(int x, int y, float radius, int type);
void set_goal(ptr_t agentPtr, ptr_t goalPtr);
void remove_goal(ptr_t ptr);

bool cpp_progress(float deltaSec);

const char* serialize();
void deserialize(const char* sp);
void go_to_frame(int f);
void update_agent(ptr_t ptr, float sz, float speed);
void update_goal(ptr_t ptr, float radius, int type);
void add_imported(const char* name, const char* text);

}