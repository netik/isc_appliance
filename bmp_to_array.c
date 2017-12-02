#include <stdio.h>
#include "libbmp/include/bitmap.h"

int main() {
  bitmap_image_t *img;
  int x,y;
  
  img = load_bmp("isc_logo.bmp");
  
  printf("w: %d h: %d\n", img->width, img->height);

  printf("bytes:\n");

  // Appears to be correct RGB values. 
  for (y = 0; y < img->height; y++) { 
    for (x = 0; x < img->width; x++) {
      printf("0x%x, ", img->pixels[x + (y * img->width)]);
    }
    printf("\n");
  }


  
}
