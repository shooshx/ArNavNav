#pragma once

#ifdef EMSCRIPTEN
typedef int ptr_t;
#else // Qt WebView
typedef long long ptr_t;
#define PTR_T long long
#endif

extern "C" {

void cpp_start();

void started_new_poly();
void added_poly_point(int x, int y);
void added_agent(int x, int y);
void moved_object(ptr_t ptr, int x, int y);

ptr_t add_goal(int x, int y);
void set_goal(ptr_t agentPtr, ptr_t goalPtr);
void remove_goal(ptr_t ptr);

void cpp_progress(float deltaSec);

const char* serialize();
void deserialize(const char* sp);
void go_to_frame(int f);

}