#include <stdlib.h>
#include <stdio.h>
#include "./quadtree.h"
#include "./collision_world.h"
#include "./intersection_event_list.h"

void construct_Quadtree(Quadtree* quadtree, double time, CollisionWorld* collisionWorld, 
                        IntersectionEventList* intersectionEventList){
  
  int num_lines = quadtree->size;
  if(num_lines == 0)
    return;
  if(num_lines <= R){
    // at a leaf node
    // sum up all collisions in this leaf node
    pairwise_line_intersection(quadtree, quadtree, collisionWorld, intersectionEventList);
  }else {  
    create_children(quadtree);
    Quadtree* upper_left  = quadtree->children[0]; 
    Quadtree* upper_right = quadtree->children[1]; 
    Quadtree* lower_left  = quadtree->children[2]; 
    Quadtree* lower_right = quadtree->children[3]; 
    Node* curr_node = quadtree->head;

    while(curr_node != NULL){
      //check if upper left
      if(is_in_bounds(curr_node->line_data, quadtree->bounds->ul, quadtree->bounds->um, quadtree->bounds->ml, quadtree->bounds->mm, time)){
          if(!upper_left->head){
            upper_left->head = curr_node;
            upper_left->tail = curr_node;
          }else{
            upper_left->tail->next = curr_node;
            upper_left->tail = curr_node;
          }
          upper_left->size = upper_left->size +1;
      }else if(is_in_bounds(curr_node->line_data, quadtree->bounds->um, quadtree->bounds->ur, quadtree->bounds->mm, quadtree->bounds->mr, time)){
          //in upper right
          if(!upper_right->head){
            upper_right->head = curr_node;
            upper_right->tail = curr_node;
          }else{
            upper_right->tail->next = curr_node;
            upper_right->tail = curr_node;
          }
          upper_right->size = upper_right->size + 1;
      }else if(is_in_bounds(curr_node->line_data, quadtree->bounds->ml, quadtree->bounds->mm, quadtree->bounds->bl, quadtree->bounds->bm, time)){
          //in lower left
          if(!lower_left->head){
            lower_left->head = curr_node;
            lower_left->tail = curr_node;
          }else{
            lower_left->tail->next = curr_node;
            lower_left->tail = curr_node;
          }
          lower_left->size = lower_left->size + 1;
      }else if(is_in_bounds(curr_node->line_data, quadtree->bounds->mm, quadtree->bounds->mr, quadtree->bounds->bm, quadtree->bounds->br, time)){
          //in lower right
          if(!lower_right->head){
            lower_right->head = curr_node;
            lower_right->tail = curr_node;
          }else{
            lower_right->tail->next = curr_node;
            lower_right->tail = curr_node;
          }
          lower_right->size = lower_right->size +1;
      }else{
        //misfits
        if(!quadtree->mis_head){
          quadtree->mis_head = curr_node;
          quadtree->mis_tail = curr_node;
        }else{
          quadtree->mis_tail->next = curr_node;
          quadtree->mis_tail = curr_node;
        }
        quadtree->mis_size = quadtree->mis_size + 1;
          // printf("MISFIT\n");
      }
      curr_node = curr_node->next;
    }

    if(upper_left->tail)
      upper_left->tail->next   = NULL;
    if(lower_left->tail)
      lower_left->tail->next   = NULL;
    if(lower_right->tail)
      lower_right->tail->next  = NULL;
    if(upper_right->tail)
      upper_right->tail->next  = NULL;
    if(quadtree->mis_tail)
      quadtree->mis_tail->next = NULL;
    pairwise_line_intersection_misfits(quadtree, collisionWorld, intersectionEventList); 
  
   // recursively constructing tree
   construct_Quadtree(upper_left,  time, collisionWorld, intersectionEventList);
   construct_Quadtree(upper_right, time, collisionWorld, intersectionEventList);
   construct_Quadtree(lower_left,  time, collisionWorld, intersectionEventList);
   construct_Quadtree(lower_right, time, collisionWorld, intersectionEventList);
  }
  return;
}

void pairwise_line_intersection(Quadtree* quadtree1, Quadtree* quadtree2, 
    CollisionWorld* collisionWorld, IntersectionEventList* intersectionEventList){
  // Test all line-line pairs in this leaf to see if they will intersect before the
  // next time step.
  Node* curr_node1 = quadtree1->head;
  Node* curr_node2 = quadtree2->head;
  Line *l1 = curr_node1->line_data;
  Line *l2 = curr_node2->line_data;
  Line* temp1;
  Line* temp2;

  while(curr_node1){
    l1 = curr_node1->line_data;
    temp1 = l1; 
    curr_node2 = curr_node1->next;
    while(curr_node2){
      l2 = curr_node2->line_data;
      temp2 = l2;
      // intersect expects compareLines(l1, l2) < 0 to be true.
      // Swap l1 and l2, if necessary.
           if (compareLines(l1, l2) >= 0) {
        temp1 = l2;
        temp2 = l1;
       // l1 = l2;
        //l2 = temp;
      }
      if(compareLines(l1,l2) != 0){
        IntersectionType intersectionType = intersect(temp1, temp2, collisionWorld->timeStep);
        if (intersectionType != NO_INTERSECTION) {
          IntersectionEventList_appendNode(intersectionEventList, temp1, temp2,
              intersectionType);
          collisionWorld->numLineLineCollisions++;
        }
      }

      curr_node2 = curr_node2->next;
    }
    curr_node1 = curr_node1->next;
  }
}

void pairwise_line_intersection_misfits(Quadtree* quadtree, 
    CollisionWorld* collisionWorld, IntersectionEventList* intersectionEventList){
  Node* curr_node = quadtree->mis_head;
  if(!curr_node)
    return;
  Line* l1 = curr_node->line_data; 
  Node* curr_node2;
  Line* l2;
  Line* temp1;
  Line* temp2;
  while(curr_node){
    l1 = curr_node->line_data;
    for(int i=0; i < 4; i++){
      curr_node2 = quadtree->children[i]->head;  
      while(curr_node2){
        l2 = curr_node2->line_data;
        temp1 = l1;
        temp2 = l2;
        // intersect expects compareLines(l1, l2) < 0 to be true.
        // Swap l1 and l2, if necessary.
        if (compareLines(l1, l2) >= 0) {
          temp1 = l2;
          temp2 = l1;
        }
        if (compareLines(temp1, temp2) != 0){
          IntersectionType intersectionType =
            intersect(temp1, temp2, collisionWorld->timeStep);
          if (intersectionType != NO_INTERSECTION) {
            IntersectionEventList_appendNode(intersectionEventList, temp1, temp2,
                intersectionType);
            collisionWorld->numLineLineCollisions++;
          }
        }
        curr_node2 = curr_node2->next;
      }

    }
    curr_node2 = curr_node->next;
    while(curr_node2){
      l2 = curr_node2->line_data;
      temp1 = l1;
      temp2 = l2;
      // intersect expects compareLines(l1, l2) < 0 to be true.
      // Swap l1 and l2, if necessary.
      if (compareLines(l1, l2) >= 0) {
        temp1 = l2;
        temp2 = l1;
      }
            if (compareLines(temp1, temp2) != 0){
        IntersectionType intersectionType =
          intersect(temp1, temp2, collisionWorld->timeStep);
        if (intersectionType != NO_INTERSECTION) {
          IntersectionEventList_appendNode(intersectionEventList, temp1, temp2,
              intersectionType);
          collisionWorld->numLineLineCollisions++;
        }
      }
      curr_node2 = curr_node2->next;
    }
    curr_node = curr_node->next;
  }
}

void create_children(Quadtree* quadtree){

  //divide into 4 
  Quadtree* upper_left = malloc_quadtree();
  init_quadtree(upper_left, quadtree->bounds->ul, quadtree->bounds->half_w, quadtree->bounds->half_h, Vec_divide(quadtree->bounds->half_w, 2), Vec_divide(quadtree->bounds->half_h, 2));
  quadtree->children[0] = upper_left;
  Quadtree* upper_right = malloc_quadtree();
  init_quadtree(upper_right, quadtree->bounds->um, quadtree->bounds->half_w, quadtree->bounds->half_h, Vec_divide(quadtree->bounds->half_w, 2), Vec_divide(quadtree->bounds->half_h, 2));
  quadtree->children[1] = upper_right;
  Quadtree* lower_left = malloc_quadtree();
  init_quadtree(lower_left, quadtree->bounds->ml, quadtree->bounds->half_w, quadtree->bounds->half_h, Vec_divide(quadtree->bounds->half_w, 2), Vec_divide(quadtree->bounds->half_h, 2));
  quadtree->children[2] = lower_left;
  Quadtree* lower_right = malloc_quadtree();
  init_quadtree(lower_right, quadtree->bounds->mm, quadtree->bounds->half_w, quadtree->bounds->half_h, Vec_divide(quadtree->bounds->half_w, 2), Vec_divide(quadtree->bounds->half_h, 2));
  quadtree->children[3] = lower_right;
}

bool is_in_bounds(Line* line, Vec ul, Vec ur, Vec bl, Vec br, double time){
  Vec p1;
  Vec p2;
  p1 = Vec_add(line->p1, Vec_multiply(line->velocity, time));
  p2 = Vec_add(line->p2, Vec_multiply(line->velocity, time));

  return (pointInParallelogram(line->p1, ul, bl, ur, br) &&
      pointInParallelogram(line->p2, ul, bl, ur, br) &&
      pointInParallelogram(p1, ul, bl, ur, br) &&
      pointInParallelogram(p2, ul, bl, ur, br));
}

Quadtree* malloc_quadtree(){
  Quadtree* quadtree = malloc(sizeof(Quadtree));
  quadtree->head = NULL;
  quadtree->tail = NULL;
  quadtree->mis_head = NULL;
  quadtree->mis_tail = NULL;

  quadtree->bounds = malloc(sizeof(Rectangle));
  return quadtree;
}

void init_quadtree(Quadtree* quadtree, Vec ul, Vec w, Vec h, Vec half_w, Vec half_h){

  quadtree->size = 0;
  quadtree->mis_size = 0;

  Vec ur = Vec_add(ul, w);
  Vec bl = Vec_add(ul, h);
  Vec br = Vec_add(ul, Vec_add(w, h));

  Vec um = Vec_add(ul, half_w);
  Vec ml = Vec_add(ul, half_h);
  Vec mr = Vec_add(ul, Vec_add(half_h, w));
  Vec bm = Vec_add(ul, Vec_add(half_w, h));
  Vec mm = Vec_add(ul, Vec_add(half_w, half_h));

  quadtree->bounds->ul = ul;
  quadtree->bounds->ur = ur;
  quadtree->bounds->bl = bl;
  quadtree->bounds->br = br; 

  quadtree->bounds->um = um;
  quadtree->bounds->ml = ml;
  quadtree->bounds->mr = mr;
  quadtree->bounds->bm = bm;
  quadtree->bounds->mm = mm;

  quadtree->bounds->w = w;
  quadtree->bounds->h = h;
  quadtree->bounds->half_w = half_w;
  quadtree->bounds->half_h = half_h;
}

void free_quadtree(Quadtree* quadtree){
  if(quadtree->children == NULL){
    // this is the case of the leaf nodes
    quadtree->head = NULL;
    quadtree->tail = NULL;
    quadtree->mis_head = NULL;
    quadtree->mis_tail = NULL;
    free_rectangle(quadtree->bounds);

  } else {
    quadtree->head = NULL;
    quadtree->tail = NULL;
    quadtree->mis_head = NULL;
    quadtree->mis_tail = NULL;
    free_rectangle(quadtree->bounds);

    free_quadtree(quadtree->children[0]);
    free_quadtree(quadtree->children[1]);
    free_quadtree(quadtree->children[2]);
    free_quadtree(quadtree->children[3]);

    quadtree->children == NULL;
  }
}

void free_linkedlist(LinkedList* linkedlist){
  // start deleteing nodes from the beginning of the list
  // Node* curr_node = linkedlist->head;
  // while(curr_node){
  //   Node* temp = curr_node->next;
  //   free_node(curr_node);
  //   curr_node = temp;
  // }
  // linked_list->head = NULL;
  // linked_list->tail = NULL;
  free(linkedlist);
}

// void free_node(Node* node){
//   node->line_data = NULL;
//   node->next = NULL;
// }


void free_rectangle(Rectangle* rectangle){
  free(rectangle);
}
void print_quadtree(Quadtree* quadtree){
  // next time step.
  Node* curr_node1 = quadtree->head;
  Line *l1 = curr_node1->line_data;
  printf("ulx: %lf uly: %lf urx: %lf ury: %lf blx: %lf bly: %lf brx: %lf bry: %lf\n", quadtree->bounds->ul.x, quadtree->bounds->ul.y, quadtree->bounds->ur.x, quadtree->bounds->ur.y, quadtree->bounds->bl.x, 
quadtree->bounds->bl.y, quadtree->bounds->br.x, quadtree->bounds->br.y); 
  while(curr_node1){
    l1 = curr_node1->line_data;
    printf("alg: L ID: %d, L {x: %lf y: %lf  x': %lf y': %lf velx: %lf vely: %lf}\n", l1->id, l1->p1.x, l1->p1.y, l1->p2.x, l1->p2.y, l1->velocity.x, l1->velocity.y); 
    curr_node1 = curr_node1->next;
  }
}
void print_quadtree_mis(Quadtree* quadtree){
  Node*  curr_node1 = quadtree->mis_head;
  while(curr_node1){
    Line* l1 = curr_node1->line_data;
    printf("mis: L ID: %d, L {x: %lf y: %lf  x': %lf y': %lf velx: %lf vely: %lf}\n", l1->id, l1->p1.x, l1->p1.y, l1->p2.x, l1->p2.y, l1->velocity.x, l1->velocity.y); 
    curr_node1 = curr_node1->next;
  }


}

