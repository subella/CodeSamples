/**
 * Copyright (c) 2012 MIT License by 6.172 Staff
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

// Implements the ADT specified in bitarray.h as a packed array of bits; a bit
// array containing bit_sz bits will consume roughly bit_sz/8 bytes of
// memory.


#include "./bitarray.h"
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#define BITS_PER_LOCATION 64
#define LSHIFT >>
#define RSHIFT <<
// ********************************* Types **********************************

// Concrete data type representing an array of bits.
struct bitarray {
  // The number of bits represented by this bit array.
  // Need not be divisible by 8.
  size_t bit_sz;

  // The underlying memory buffer that stores the bits in
  // packed form (8 per byte).
  uint64_t* buf;
};

static const uint64_t reverse_table[] = {
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
// ******************** Prototypes for static functions *********************

// Rotates a subarray left by an arbitrary number of bits.
//
// bit_offset is the index of the start of the subarray
// bit_length is the length of the subarray, in bits
// bit_left_amount is the number of places to rotate the
//                    subarray left
//
// The subarray spans the half-open interval
// [bit_offset, bit_offset + bit_length)
// That is, the start is inclusive, but the end is exclusive.
static void bitarray_rotate_left(bitarray_t* const bitarray,
                                 const size_t bit_offset,
                                 const size_t bit_length,
                                 const size_t bit_left_amount);

// Rotates a subarray left by one bit.
//
// bit_offset is the index of the start of the subarray
// bit_length is the length of the subarray, in bits
//
// The subarray spans the half-open interval
// [bit_offset, bit_offset + bit_length)
// That is, the start is inclusive, but the end is exclusive.
static void bitarray_rotate_left_one(bitarray_t* const bitarray,
                                     const size_t bit_offset,
                                     const size_t bit_length);

// Portable modulo operation that supports negative dividends.
//
// Many programming languages define modulo in a manner incompatible with its
// widely-accepted mathematical definition.
// http://stackoverflow.com/questions/1907565/c-python-different-behaviour-of-the-modulo-operation
// provides details; in particular, C's modulo
// operator (which the standard calls a "remainder" operator) yields a result
// signed identically to the dividend e.g., -1 % 10 yields -1.
// This is obviously unacceptable for a function which returns size_t, so we
// define our own.
//
// n is the dividend and m is the divisor
//
// Returns a positive integer r = n (mod m), in the range
// 0 <= r < m.
static size_t modulo(const ssize_t n, const size_t m);

// Produces a mask which, when ANDed with a byte, retains only the
// bit_index th byte.
//
// Example: bitmask(5) produces the byte 0b00100000.
//
// (Note that here the index is counted from right
// to left, which is different from how we represent bitarrays in the
// tests.  This function is only used by bitarray_get and bitarray_set,
// however, so as long as you always use bitarray_get and bitarray_set
// to access bits in your bitarray, this reverse representation should
// not matter.
static inline uint64_t bitmask(const size_t bit_index);

// Accepts a start and end point and reverses all the bits inside
// calls on helper methods to reverse locaitons and tails
static inline void bitarray_reverse(bitarray_t* const bitarray,
                            const size_t initial_start,
                            const size_t initial_end);

// Takes in a start and end location that are full 64 bit bocks
// that we want to reverse. Revereses the bits with a lookup
// table and then swaps the blocks from outside in
static inline void bitarray_reverse_location(bitarray_t* const bitarray,
                                      uint64_t *bitarray_ptr,
                                      const int initial_start_location,
                                      const int initial_end_location,
                                      const size_t initial_start,
                                      const size_t inital_end);

// Reverses the bits in a location using a lookup table of 8 bits
static void inline bitarray_reverse_location_bits(uint64_t *bitarray_ptr, int location);

// Swaps the left and right tail in place since they are the same length
static void inline bitarray_swap_tails_left_equal_right(bitarray_t* const bitarray,
                                          uint64_t *bitarray_ptr,
                                          const int initial_start_location,
                                          const int initial_end_location,
                                          const uint64_t left_bits_length,
                                          const uint64_t right_bits_length);

// Left tail is greater than right so we save left tail and store it in the right
// tail location. Then iterate through all the blocks and shift along that number
// of bits until locations have all been shifted down. Adds the final tail into the
// right tail
static void inline bitarray_swap_tails_left_greater_right(bitarray_t* const bitarray,
                                                          uint64_t *bitarray_ptr,
                                                          const int initial_start_location,
                                                          const int initial_end_location,
                                                          const uint64_t left_bits_length,
                                                          const uint64_t right_bits_length);

// Same as left tail greater than right except in the other direction
// for tail swapping and bit moving
static void inline bitarray_swap_tails_left_less_right(bitarray_t* const bitarray,
                                                          uint64_t *bitarray_ptr,
                                                          const int initial_start_location,
                                                          const int initial_end_location,
                                                          const uint64_t left_bits_length,
                                                          const uint64_t right_bits_length);
// ******************************* Functions ********************************

inline bitarray_t* bitarray_new(const size_t bit_sz) {
  // Allocate an underlying buffer of ceil(bit_sz/8) bytes.
  uint64_t* const buf = (uint64_t*) calloc(1, (bit_sz + BITS_PER_LOCATION - 1)/ BITS_PER_LOCATION * 8);
  if (buf == NULL) {
    return NULL;
  }

  // Allocate space for the struct.
  bitarray_t* const bitarray = malloc(sizeof(struct bitarray));
  if (bitarray == NULL) {
    free(buf);
    return NULL;
  }

  bitarray->buf = buf;
  bitarray->bit_sz = bit_sz;
  return bitarray;
}

void inline bitarray_free(bitarray_t* const bitarray) {
  if (bitarray == NULL) {
    return;
  }
  free(bitarray->buf);
  bitarray->buf = NULL;
  free(bitarray);
}

size_t bitarray_get_bit_sz(const bitarray_t* const bitarray) {
  return bitarray->bit_sz;
}

bool inline bitarray_get(const bitarray_t* const bitarray, const size_t bit_index) {
  assert(bit_index < bitarray->bit_sz);

  // We're storing bits in packed form, 8 per byte.  So to get the nth
  // bit, we want to look at the (n mod 8)th bit of the (floor(n/8)th)
  // byte.
  //
  // In C, integer division is floored explicitly, so we can just do it to
  // get the byte; we then bitwise-and the byte with an appropriate mask
  // to produce either a zero byte (if the bit was 0) or a nonzero byte
  // (if it wasn't).  Finally, we convert that to a boolean.
  return (bitarray->buf[bit_index / BITS_PER_LOCATION] & bitmask(bit_index)) ?
         true : false;
}

void inline bitarray_set(bitarray_t* const bitarray,
                  const size_t bit_index,
                  const bool value) {
  assert(bit_index < bitarray->bit_sz);

  // We're storing bits in packed form, 8 per byte.  So to set the nth
  // bit, we want to set the (n mod 8)th bit of the (floor(n/8)th) byte.
  //
  // In C, integer division is floored explicitly, so we can just do it to
  // get the byte; we then bitwise-and the byte with an appropriate mask
  // to clear out the bit we're about to set.  We bitwise-or the result
  // with a byte that has either a 1 or a 0 in the correct place.
  bitarray->buf[bit_index / BITS_PER_LOCATION] =
    (bitarray->buf[bit_index / BITS_PER_LOCATION] & ~bitmask(bit_index)) |
    (value ? bitmask(bit_index) : 0);
}

void inline bitarray_randfill(bitarray_t* const bitarray) {
  int32_t *ptr = (int32_t *) bitarray->buf;
  for (int64_t i = 0; i < bitarray->bit_sz / 32 + 1; i++) {
    ptr[i] = rand();
  }
}

static void inline bitarray_mask_subsection(bitarray_t* const bitarray,
                     const size_t bit_offset,
                     const size_t bit_length,
                     const ssize_t bit_right_amount) {
}

static inline void bitarray_reverse_bits(bitarray_t* const bitarray,
                     const size_t initial_start,
                     const size_t initial_end) {
  size_t start = initial_start;
  size_t end = initial_end;
  bool start_value;
  bool end_value;
  // swaps the first bit with the last bit from the outside in
  // until no bits left unswapped
  while (start < end) {
    start_value = bitarray_get(bitarray, start);
    end_value = bitarray_get(bitarray, end);
    if (start_value ^ end_value) {
      bitarray_set(bitarray, start, end_value);
      bitarray_set(bitarray, end, start_value);
    }
    start++;
    end--;
  }
}

static inline void bitarray_reverse(bitarray_t* const bitarray,
                     const size_t initial_start,
                     const size_t initial_end) {
  int const initial_start_location = initial_start / BITS_PER_LOCATION;
  int const initial_end_location = initial_end / BITS_PER_LOCATION;
  uint64_t left_bits_length  = (BITS_PER_LOCATION*(initial_start_location + 1) - initial_start) % 64;
  if (left_bits_length == 0)
     left_bits_length = 64;
  uint64_t right_bits_length = (initial_end - BITS_PER_LOCATION*(initial_end_location) + 1) % 64;
  if (right_bits_length == 0)
     right_bits_length = 64;
  uint64_t *bitarray_ptr =  bitarray->buf;
  // if bitarray is small just loop through the bits and switch them
  if (initial_end - initial_start <= 2 * BITS_PER_LOCATION) {
    bitarray_reverse_bits(bitarray, initial_start, initial_end);
    return;
  }
  // calls function to reverse whole locations

  bitarray_reverse_location(bitarray, bitarray_ptr, initial_start_location, initial_end_location,
                            initial_start, initial_end);
  // reverses bits in tails
  if (left_bits_length != 0)
    bitarray_reverse_bits(bitarray, initial_start, initial_start + left_bits_length - 1);
  if (right_bits_length != 0)
    bitarray_reverse_bits(bitarray, initial_end - right_bits_length + 1, initial_end);

  if (left_bits_length == right_bits_length) {
    // call function to swap tails and move bits
    bitarray_swap_tails_left_equal_right(bitarray, bitarray_ptr,
                                        initial_start_location, initial_end_location,
                                        left_bits_length, right_bits_length);

  } else if (left_bits_length > right_bits_length) {
    // call function to swap tails and move bits
    bitarray_swap_tails_left_greater_right(bitarray, bitarray_ptr,
                                        initial_start_location, initial_end_location,
                                        left_bits_length, right_bits_length);

  } else {
    // call function to swap tails and move bits
    bitarray_swap_tails_left_less_right(bitarray, bitarray_ptr,
                                        initial_start_location, initial_end_location,
                                        left_bits_length, right_bits_length);
  }
}

static inline void bitarray_reverse_location(bitarray_t* const bitarray,
                                      uint64_t *bitarray_ptr,
                                      const int initial_start_location,
                                      const int initial_end_location,
                                      const size_t initial_start,
                                      const size_t initial_end) {
  int start_location = initial_start_location + 1;
  int end_location = initial_end_location - 1;
  uint64_t temp;

// reverses whole locations from the outside in
  while (start_location < end_location) {
    bitarray_reverse_location_bits(bitarray_ptr, start_location);
    bitarray_reverse_location_bits(bitarray_ptr, end_location);
    temp = bitarray_ptr[start_location];
    bitarray_ptr[start_location] =  bitarray_ptr[end_location];
    bitarray_ptr[end_location] = temp;
    start_location++;
    end_location--;
  }

  // reverse the middle location
  if (start_location == end_location) {
    bitarray_reverse_location_bits(bitarray_ptr, start_location);
  }
}

static inline void bitarray_reverse_location_bits(uint64_t* bitarray_ptr, int location) {
    uint64_t temp = bitarray_ptr[location];
    bitarray_ptr[location] =   (reverse_table[temp & 0xff] RSHIFT 56) |
                                   (reverse_table[(temp LSHIFT 8) & 0xff]  RSHIFT 48) |
                                   (reverse_table[(temp LSHIFT 16) & 0xff] RSHIFT 40) |
                                   (reverse_table[(temp LSHIFT 24) & 0xff] RSHIFT 32) |
                                   (reverse_table[(temp LSHIFT 32) & 0xff] RSHIFT 24) |
                                   (reverse_table[(temp LSHIFT 40) & 0xff] RSHIFT 16) |
                                   (reverse_table[(temp LSHIFT 48) & 0xff] RSHIFT  8) |
                                   (reverse_table[(temp LSHIFT 56) & 0xff]);

    return;
}


static inline void bitarray_swap_tails_left_equal_right(bitarray_t* const bitarray,
                    uint64_t *bitarray_ptr,
                    const int initial_start_location,
                    const int initial_end_location,
                    const uint64_t left_bits_length,
                    const uint64_t right_bits_length) {
    // save entire right tail
    uint64_t right_bits_bak = bitarray_ptr[initial_end_location];
    uint64_t left_bits_extra = bitarray_ptr[initial_start_location]
                              LSHIFT(BITS_PER_LOCATION - left_bits_length);
    if (right_bits_length == 64) {
      bitarray_ptr[initial_end_location] = 0;
    } else {
      bitarray_ptr[initial_end_location] = ((bitarray_ptr[initial_end_location]
                              LSHIFT(right_bits_length)) RSHIFT(right_bits_length));
    }
    bitarray_ptr[initial_end_location] |= left_bits_extra;

    uint64_t right_bits_extra = right_bits_bak RSHIFT(BITS_PER_LOCATION - right_bits_length);
    if (left_bits_length == 64) {
      bitarray_ptr[initial_start_location] = 0;
    } else {
      bitarray_ptr[initial_start_location] = ((bitarray_ptr[initial_start_location]
                              RSHIFT(left_bits_length)) LSHIFT(left_bits_length));
    }
    bitarray_ptr[initial_start_location] |= right_bits_extra;
}

static inline void bitarray_swap_tails_left_greater_right(bitarray_t* const bitarray,
                    uint64_t *bitarray_ptr,
                    const int initial_start_location,
                    const int initial_end_location,
                    const uint64_t left_bits_length,
                    const uint64_t right_bits_length) {
  // left shift everything by (left_bits_length - right_bits_length)
  // right tail key bits
  uint64_t right_bits = (bitarray_ptr[initial_end_location]
                                RSHIFT(BITS_PER_LOCATION - right_bits_length))
                                LSHIFT(left_bits_length - right_bits_length);
  // change right tail to have zeros in right key bits
  bitarray_ptr[initial_end_location] = (bitarray_ptr[initial_end_location]
                                LSHIFT(right_bits_length))
                                RSHIFT(right_bits_length);
  // capture left key bits to store in right tail location
  uint64_t left_in_right = bitarray_ptr[initial_start_location]
                                LSHIFT(BITS_PER_LOCATION - right_bits_length);
  // combine all bits that will fit in right tail to right tail
  bitarray_ptr[initial_end_location] |= left_in_right;
  // capture excess bits that didnt fit in right tail
  uint64_t left_bits_middle = (bitarray_ptr[initial_start_location]
                                LSHIFT(BITS_PER_LOCATION - left_bits_length))
                                RSHIFT(BITS_PER_LOCATION - left_bits_length + right_bits_length);
  uint64_t left_bits_middle_length = left_bits_length - right_bits_length;
  if (left_bits_length == 64) {
    bitarray_ptr[initial_start_location] = 0;
  } else {
    bitarray_ptr[initial_start_location] = (bitarray_ptr[initial_start_location]
                                RSHIFT(left_bits_length)) LSHIFT(left_bits_length);
  }
  bitarray_ptr[initial_start_location] |= right_bits;
  uint64_t left_data;
  for (int i = initial_start_location + 1; i < initial_end_location; i++) {
    // get bits we want to transfer to previous location
    left_data = bitarray_ptr[i] RSHIFT(BITS_PER_LOCATION - left_bits_middle_length);
    // prepare location for next insertion
    bitarray_ptr[i] = bitarray_ptr[i] LSHIFT(left_bits_middle_length);
    // transfer bits to previous location
    bitarray_ptr[i - 1] |= left_data;
  }

  // fix final location
  bitarray_ptr[initial_end_location - 1] |= left_bits_middle;
}

static inline void bitarray_swap_tails_left_less_right(bitarray_t* const bitarray,
                    uint64_t *bitarray_ptr,
                    const int initial_start_location,
                    const int initial_end_location,
                    const uint64_t left_bits_length,
                    const uint64_t right_bits_length) {
  uint64_t left_bits = (bitarray_ptr[initial_start_location]
                                LSHIFT(BITS_PER_LOCATION - left_bits_length))
                                RSHIFT(right_bits_length - left_bits_length);
  bitarray_ptr[initial_start_location] = (bitarray_ptr[initial_start_location]
                                RSHIFT(left_bits_length))
                                LSHIFT(left_bits_length);
  uint64_t right_in_left = bitarray_ptr[initial_end_location]
                                RSHIFT(BITS_PER_LOCATION - left_bits_length);
  bitarray_ptr[initial_start_location] |= right_in_left;
  uint64_t right_bits_middle = (bitarray_ptr[initial_end_location]
                                RSHIFT(BITS_PER_LOCATION - right_bits_length))
                                LSHIFT(BITS_PER_LOCATION - right_bits_length + left_bits_length);
  uint64_t right_bits_middle_length = right_bits_length - left_bits_length;

  if (right_bits_length == 64) {
      bitarray_ptr[initial_end_location] = 0;
  } else {
      bitarray_ptr[initial_end_location] = (bitarray_ptr[initial_end_location]
                                LSHIFT(right_bits_length))
                                RSHIFT(right_bits_length);
  }

  bitarray_ptr[initial_end_location] |= left_bits;
  uint64_t right_data;
  for (int i = initial_end_location - 1; i > initial_start_location; i--) {
    // get bits we want to transfer to previous location
    right_data = bitarray_ptr[i] LSHIFT(BITS_PER_LOCATION - right_bits_middle_length);
    // prepare location for next insertion
    bitarray_ptr[i] = bitarray_ptr[i] RSHIFT right_bits_middle_length;
    // transfer bits to previous location
    bitarray_ptr[i + 1] |= right_data;
  }
  // fix final location
  bitarray_ptr[initial_start_location + 1] |= right_bits_middle;
}
inline void bitarray_rotate(bitarray_t* const bitarray,
                     const size_t bit_offset,
                     const size_t bit_length,
                     const ssize_t bit_right_amount) {
  assert(bit_offset + bit_length <= bitarray->bit_sz);
  size_t pivot;
  if (bit_length == 0) {
    return;
  }
  if (modulo(bit_right_amount, bit_length) == 0) {
     return;
  }
  if (bit_right_amount > 0) {
    pivot = bit_offset - modulo(bit_right_amount, bit_length) - 1 + bit_length;
  } else {
    pivot = bit_offset + modulo(-bit_right_amount, bit_length) - 1;
  }

  // reverse left array, then right array, then combined
  bitarray_reverse(bitarray, bit_offset, pivot);
  bitarray_reverse(bitarray, pivot + 1, bit_offset + bit_length - 1);
  bitarray_reverse(bitarray, bit_offset, bit_offset + bit_length - 1);
}

static inline void bitarray_rotate_left(bitarray_t* const bitarray,
                                 const size_t bit_offset,
                                 const size_t bit_length,
                                 const size_t bit_left_amount) {
  for (size_t i = 0; i < bit_left_amount; i++) {
    bitarray_rotate_left_one(bitarray, bit_offset, bit_length);
  }
}

static inline void bitarray_rotate_left_one(bitarray_t* const bitarray,
                                     const size_t bit_offset,
                                     const size_t bit_length) {
  // Grab the first bit in the range, shift everything left by one, and
  // then stick the first bit at the end.
  const bool first_bit = bitarray_get(bitarray, bit_offset);
  size_t i;
  for (i = bit_offset; i + 1 < bit_offset + bit_length; i++) {
    bitarray_set(bitarray, i, bitarray_get(bitarray, i + 1));
  }
  bitarray_set(bitarray, i, first_bit);
}

static inline size_t modulo(const ssize_t n, const size_t m) {
  const ssize_t signed_m = (ssize_t)m;
  assert(signed_m > 0);
  const ssize_t result = ((n % signed_m) + signed_m) % signed_m;
  assert(result >= 0);
  return (size_t)result;
}

static inline uint64_t bitmask(const size_t bit_index) {
  return (uint64_t)1 << (bit_index % BITS_PER_LOCATION);
}
