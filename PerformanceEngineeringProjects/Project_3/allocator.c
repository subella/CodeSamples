/**
 * Copyright (c) 2015 MIT License by 6.172 Staff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 **/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "./allocator_interface.h"
#include "./memlib.h"

// Don't call libc malloc!
#define malloc(...) (USE_MY_MALLOC)
#define free(...) (USE_MY_FREE)
#define realloc(...) (USE_MY_REALLOC)

// All blocks must have a specified minimum alignment.
// The alignment requirement (from config.h) is >= 8 bytes.
#ifndef ALIGNMENT
  #define ALIGNMENT 8
#endif

// Rounds up to the nearest multiple of ALIGNMENT.
#define ALIGN(size) (((uint64_t)(size) + (ALIGNMENT-1)) & ~(ALIGNMENT-1))
#define ALIGN_BINS(size) (1<<get_bin_index(size))

// The smallest aligned size that will hold a size_t value.
// #define SIZE_T_SIZE (ALIGN(sizeof(size_t)))
#define FREE_LIST_T_SIZE (ALIGN(sizeof(free_list_t)))

#define NUM_BINS 32


typedef struct free_list_t {
  struct free_list_t* next;
  //struct free_list_t* prev;
  int32_t size;
  //int32_t prev_size;
} free_list_t;

// To start, I think bins list should hold about 30 different free lists
// Each free list will hold a power of 2
// ie index 0 holds sizes < 2**0, index 5 holds sizes < 2**5
free_list_t* bins_list[NUM_BINS];
//int32_t last_mem_size = NULL;
//free_list_t* end_block = NULL;

// constants used to calculate log2 of a 64 bit number
const uint64_t deBruijn = 0x022fdd63cc95386d;
const int convert[64] = {
  0, 1, 2, 53, 3, 7, 54, 27,
  4, 38, 41, 8, 34, 55, 48, 28,
  62, 5, 39, 46, 44, 42, 22, 9,
  24, 35, 59, 56, 49, 18, 29, 11,
  63, 52, 6, 26, 37, 40, 33, 47,
  61, 45, 43, 21, 23, 58, 17, 10,
  51, 25, 36, 32, 60, 20, 57, 16,
  50, 31, 19, 15, 30, 14, 13, 12
};

int log2_64(uint64_t v) {
  // rounds v up to nearest power of 2
  // ie. smallest possible bin that can contain it
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v |= v >> 32;
  v++;
  // courtesy of lecture notes
  // uses lookup table to compute log2 in constant time
  return convert[(v * deBruijn) >> 58];
}

// returns the index of the free list that matches this size
static inline int get_bin_index(uint64_t size) {
  assert(log2_64(size) < NUM_BINS);
  return log2_64(size);
}

static inline uint8_t check_is_available(free_list_t* free_list) {
  return free_list->size & 1;
}

static inline free_list_t* break_up_free_list(int curr_index, int goal_index) {
  free_list_t* p = bins_list[curr_index];
  bins_list[curr_index] = (bins_list[curr_index])->next;
  // this loop breaks up a larger free list, storing halves into other free lists
  // on its way down.
  // 128 would break up to do 64 halves. One of those halves is placed in a 64 bit free list. The
  // other is broken down again
  while (curr_index > goal_index) {
    // divide the block in half by putting it into a the previous linked list
    p->next = bins_list[curr_index - 1];
    // now we have 2 halfs in the previous free list
    bins_list[curr_index - 1] = p;
    // gets the rightmost half, moves the pointer up the corresponding power of 2
    p = (free_list_t*)((char*)p + (int)(1<<(curr_index-1)));
    curr_index--;
  }
  return p;
}

// adds a new block to the front of the free list that it belongs to
static inline void add_to_free_list(free_list_t* block, index) {
  if (bins_list[index] != NULL) {
    block->next = bins_list[index];
    //block->prev = NULL;
    //bins_list[index]->prev = block;
    bins_list[index] = block;
  } else {
    block->next = bins_list[index];
    //block->prev = NULL;
    bins_list[index] = block;
  }
}

// removes a block from the free list
//static inline void remove_from_free_list(free_list_t* block) {
//  // If the block has nothing for prev, set the bins list of
//  // that size to point to it's next. If it has a prev, set
//  // that block's next to this block's next to remove it
//  if (block->next != NULL) {
//      block->next->prev = block->prev;
//    }
//  if (block->prev == NULL) {
//    int index = get_bin_index(block->size);
//    bins_list[index] = block->next;
//  } else {
//    block->prev->next = block->next;
//  }
//}

//static inline free_list_t* coalesce_blocks(free_list_t** block_left, free_list_t** block_right) {
//  remove_from_free_list(*block_left);
//  remove_from_free_list(*block_right);
//  int index = get_bin_index(*block_left) + 1;
//  add_to_free_list(*block_left, index);
//  return *block_left;
//}

// check - This checks our invariant that the size_t header before every
// block points to either the beginning of the next block, or the end of the
// heap.
int my_check() {
  char* p;
  char* lo = (char*)mem_heap_lo();
  char* hi = (char*)mem_heap_hi() + 1;
  size_t size = 0;

  p = lo;
  while (lo <= p && p < hi) {
    size = ALIGN(((free_list_t*)p)->size + FREE_LIST_T_SIZE);
    p += size;
  }

  if (p != hi) {
    printf("Bad headers did not end at heap_hi!\n");
    printf("heap_lo: %p, heap_hi: %p, size: %lu, p: %p\n", lo, hi, size, p);
    return -1;
  }

  for (int i = 0; i < NUM_BINS; i++) {
    free_list_t* free_list = bins_list[i];
    // printf("free_list: %p\n", free_list);
    while (free_list) {
      // printf("hello");
      int32_t size = free_list->size;
      assert(index == get_bin_index(size));
      // printf("Bin Size:%d Size: %d, Index: %d\n", (1<<(int)i), size, i);
      free_list = free_list->next;
    }
  }
  return 0;
}

// init - Initialize the malloc package.  Called once before any other
// calls are made.  Since this is a very simple implementation, we just
// return success.
int my_init() {
  // initialize each free list in our bins list to null
  for (int i = 0; i < NUM_BINS; i++) {
    bins_list[i] = NULL;
  }
  return 0;
}

//  malloc - Allocate a block by incrementing the brk pointer.
//  Always allocate a block whose size is a multiple of the alignment.
void* my_malloc(size_t size) {
  // by definition return NULL if size is 0
  if (size == 0) {
    return NULL;
  }

  free_list_t* p;
  // We allocate a little bit of extra memory so that we can store the
  // size of the block we've allocated.  Take a look at realloc to see
  // one example of a place where this can come in handy.
  int aligned_size = ALIGN(size + FREE_LIST_T_SIZE);

  // get which free list in our bin list that this size corresponds to
  int bin_index = get_bin_index(aligned_size);
  // assign our current working free list to the calculated index
  free_list_t* curr_free_list = bins_list[bin_index];
  // if our current free list has blocks available, we should use them.
  // if it doesn't, try and break up larger free lists
  if (curr_free_list != NULL) {
  // printf("not null");
    free_list_t* head = bins_list[bin_index];
    bins_list[bin_index] = bins_list[bin_index]->next;

    // update the size metadata
    head->size = size & ~1;
    // return pointer to start of allocated data
    return (void*)((char*)head + FREE_LIST_T_SIZE);
  } else {
    // TODO get rid of this for loop (keep track of which lists are not NULL)
    for (int i = index + 1; i < NUM_BINS; i++) {
      // obtain the next smallest free list with available blocks
      if (bins_list[i] != NULL) {
        p = break_up_free_list(i, index);
        // update p metadata
        p->size = size & ~1;
        // return pointer to data start
        return (void*)((char*)p + FREE_LIST_T_SIZE);
      }
    }
  }

  // Expands the heap by the given number of bytes and returns a pointer to
  // the newly-allocated area.  This is a slow call, so you will want to
  // make sure you don't wind up calling it on every malloc.
  aligned_size = ALIGN_BINS(size + FREE_LIST_T_SIZE);
  p = mem_sbrk(aligned_size);
  //p->prev_size = last_mem_size;
  //last_mem_size = size;
  //end_block = p;

  if (p == (void*) - 1) {
    // Whoops, an error of some sort occurred.  We return NULL to let
    // the client code know that we weren't able to allocate memory.
    return NULL;
  } else {
    // We store the size of the block we've allocated in the first
    // FREE_LIST_T_SIZE bytes.

    p->size = size & ~1;

    // Then, we return a pointer to the rest of the block of memory,
    // which is at least size bytes long.  We have to cast to uint8_t
    // before we try any pointer arithmetic because voids have no size
    // and so the compiler doesn't know how far to move the pointer.
    // Since a uint8_t is always one byte, adding FREE_LIST_T_SIZE after
    // casting advances the pointer by FREE_LIST_T_SIZE bytes.
    return (void*)((char*)p + FREE_LIST_T_SIZE);
  }
}

// free - Freeing a block does nothing.
void my_free(void* ptr) {
  if (ptr == NULL) {
    return;
  }
  // get size of allocated data
  free_list_t* block_start = ((char*)ptr - FREE_LIST_T_SIZE);
  int aligned_size = ALIGN(block_start->size + FREE_LIST_T_SIZE);
  int index = get_bin_index(aligned_size);

  add_to_free_list(block_start, index);
  // makes block available to be coalesced
  block_start->size |= 1;
}

// realloc - Implemented simply in terms of malloc and free
void* my_realloc(void* ptr, size_t size) {
  void* newptr;
  int copy_size;
  int new_size;
  // if ptr is NULL this is the same as my_malloc(size)
  if (ptr == NULL) {
    return my_malloc(size);
  }
  // if size is 0 this is the same as my_free(size)
  if (size == 0) {
    my_free(size);
    return NULL;
  }


  // Get the size of the old block of memory.  Take a peek at my_malloc(),
  // where we stashed this in the FREE_LIST_T_SIZE bytes directly before the
  // address we returned.  Now we can back up by that many bytes and read
  // the size.
  free_list_t* block_start = ((char*)ptr - FREE_LIST_T_SIZE);
  copy_size = ALIGN(block_start->size + FREE_LIST_T_SIZE);
  new_size = ALIGN(size + FREE_LIST_T_SIZE);

  // If the new block is in the same bin as the old one, we just need to change
  // size
  if (get_bin_index(copy_size) == get_bin_index(new_size)){
    block_start->size = size & ~1;
    return ptr;
  }

  int copy_bin_size = ALIGN_BINS(copy_size);
  if (((char*)block_start + copy_bin_size) == ((char*) mem_heap_hi() + 1)) {
    int new_bin_size = ALIGN_BINS(new_size);
    block_start->size = size & ~1;
    mem_sbrk(new_bin_size - copy_bin_size);
    return ptr;
  }



  // Allocate a new chunk of memory, and fail if that allocation fails.
  newptr = my_malloc(size);
    if (newptr == NULL) {
          return NULL;
    }
  // If the new block is smaller than the old one, we have to stop copying
  // early so that we don't write off the end of the new block of memory.
  if (size < copy_size) {
    copy_size = size;
  }

  // This is a standard library call that performs a simple memory copy.
  memcpy(newptr, ptr, copy_size);

  // Release the old block.
  my_free(ptr);

  // Return a pointer to the new block.
  return newptr;
}
