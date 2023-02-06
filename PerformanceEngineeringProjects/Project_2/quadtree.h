#ifndef QUADTREE_H_
#define QUADTREE_H_

#include "./line.h"
#include "./vec.h"
#include "./collision_world.h"
#include "./intersection_event_list.h"

#define R 18

typedef struct Rectangle{
  //four corners
  Vec ul; 
  Vec ur;
  Vec bl;
  Vec br;

  //midpoints
  Vec um;
  Vec ml;
  Vec mm; 
  Vec mr;
  Vec bm;
  //width and heights
  Vec w; 
  Vec h; 
  Vec half_w;
  Vec half_h;

} Rectangle;


typedef struct Node{
  Line* line_data;
  struct Node* next;
  int idx;
} Node;

typedef struct LinkedList{
  Node* head;
  Node* tail;
} LinkedList;

typedef struct Quadtree{
  Node* head;
  Node* tail;
  Node* mis_head;
  Node* mis_tail;
  int size;
  int mis_size;
  Rectangle* bounds; 
  struct Quadtree* children[4]; 

} Quadtree;


void partition_quadtree(Quadtree* quadtree, Vec ul, Vec width, Vec height);

void construct_Quadtree(Quadtree* quadtree, double time, CollisionWorld* collisionWorld, 
    IntersectionEventList* intersectionEventList);

bool is_in_quadrant(Quadtree* quadtree, Line* line, double time);

bool is_in_bounds(Line* line, Vec ul, Vec ur, Vec br, Vec bl, double time);

Quadtree* malloc_quadtree();

void init_quadtree(Quadtree* quadtree, Vec ul, Vec w, Vec h, Vec half_w, Vec half_h);

void create_children(Quadtree* quadtree);

void pairwise_line_intersection(Quadtree* quadtree1, Quadtree* quadtree2, 
    CollisionWorld* collisionWorld, 
    IntersectionEventList* intersectionEventList);

void pairwise_line_intersection_misfits(Quadtree* quadtree, 
    CollisionWorld* collisionWorld, 
    IntersectionEventList* intersectionEventList);

void free_rectangle(Rectangle* rectangle);


void free_linkedlist(LinkedList* linkedlist);

void free_quadtree(Quadtree* quadtree);

void print_quadtree(Quadtree* quadtree);

void print_quadtree_mis(Quadtree* quadtree);

#endif
