/***********************************************************************
Test case for the C implementation of the convex hull algorithm
***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "miallib.h"
#include "myhull_c.h"

// Visual representation of hull points
void print_hull_visual(IMAGE* hull_img, int nx, int ny) {
    UCHAR* pout = (UCHAR*)GetImPtr(hull_img);
    
    printf("Visual representation of the hull (10 = hull points):\n");
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            int pos = y * nx + x;
            if (pout[pos] == 10) {
                printf("X ");  // Hull point
            } else if (pout[pos] == 9) {
                printf("O ");  // Border point
            } else if (pout[pos] > 0) {
                printf(". ");  // Other marked point
            } else {
                printf("  ");  // Empty
            }
        }
        printf("\n");
    }
}

// Test the _chainHull_2D function directly
void test_chain_hull_2d() {
    printf("Testing chainHull_2D function...\n");
    
    // Create a set of test points - a simple square with some interior points
    intpair_t points[] = {
        {1, 1}, {5, 1}, {5, 5}, {1, 5},  // Corners of a square
        {2, 2}, {3, 3}, {4, 2}, {2, 4}   // Interior points
    };
    int num_points = sizeof(points) / sizeof(points[0]);
    
    // Print input points
    printf("Input points:\n");
    for (int i = 0; i < num_points; i++) {
        printf("(%d, %d) ", points[i].a, points[i].b);
    }
    printf("\n");
    
    // Allocate memory for hull - should be at least as large as input
    intpair_t* hull = (intpair_t*)malloc((num_points + 1) * sizeof(intpair_t));
    if (!hull) {
        printf("Failed to allocate memory for hull\n");
        return;
    }
    
    // Compute the convex hull
    int hull_size = _chainHull_2D(points, num_points, hull);
    
    // Print hull points
    printf("Hull points:\n");
    for (int i = 0; i < hull_size; i++) {
        printf("(%d, %d) ", hull[i].a, hull[i].b);
    }
    printf("\n");
    
    // Verify the hull (should be the four corner points)
    assert(hull_size == 4);
    int has_point_1_1 = 0;
    int has_point_5_1 = 0;
    int has_point_5_5 = 0;
    int has_point_1_5 = 0;
    
    for (int i = 0; i < hull_size; i++) {
        if (hull[i].a == 1 && hull[i].b == 1) has_point_1_1 = 1;
        if (hull[i].a == 5 && hull[i].b == 1) has_point_5_1 = 1;
        if (hull[i].a == 5 && hull[i].b == 5) has_point_5_5 = 1;
        if (hull[i].a == 1 && hull[i].b == 5) has_point_1_5 = 1;
    }
    
    assert(has_point_1_1 && has_point_5_1 && has_point_5_5 && has_point_1_5);
    free(hull);
    printf("chainHull_2D test passed!\n");
}

// Test the chull function on an image
void test_chull_on_image() {
    printf("\nTesting chull function on an image...\n");
    
    // Create a test image (20x20 with a simple shape)
    int nx = 20;
    int ny = 20;
    IMAGE* test_img = create_image(t_UINT32, nx, ny, 1);
    if (!test_img) {
        fprintf(stderr, "Failed to create test image\n");
        return;
    }
    
    // Fill image with background (0)
    UINT32* pimg = (UINT32*)GetImPtr(test_img);
    memset(pimg, 0, nx * ny * sizeof(UINT32));
    
    // Draw a shape (label 1) - an L shape
    for (int y = 5; y < 15; y++) {
        for (int x = 5; x < 10; x++) {
            pimg[y * nx + x] = 1;
        }
    }
    for (int y = 10; y < 15; y++) {
        for (int x = 10; x < 15; x++) {
            pimg[y * nx + x] = 1;
        }
    }
    
    // Print the original image
    printf("Original image (label 1 = L shape):\n");
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            printf("%s", (pimg[y * nx + x] > 0 ? "# " : ". "));
        }
        printf("\n");
    }
    
    // Compute the convex hull (8-connectivity)
    IMAGE* hull_img = _chull(test_img, 8);
    if (!hull_img) {
        fprintf(stderr, "Failed to compute hull\n");
        free_image(test_img);
        return;
    }
    
    // Print the hull
    print_hull_visual(hull_img, nx, ny);
    
    // Free memory
    free_image(test_img);
    free_image(hull_img);
    
    printf("chull test completed!\n");
}

// Test with real-world scenarios
void test_real_world_cases() {
    printf("\nTesting with more complex shapes...\n");
    
    // Create a test image (30x30 with multiple shapes)
    int nx = 30;
    int ny = 30;
    IMAGE* test_img = create_image(t_UINT32, nx, ny, 1);
    if (!test_img) {
        fprintf(stderr, "Failed to create test image\n");
        return;
    }
    
    // Fill image with background (0)
    UINT32* pimg = (UINT32*)GetImPtr(test_img);
    memset(pimg, 0, nx * ny * sizeof(UINT32));
    
    // Draw shape 1 (label 1) - a star-like shape
    int star_points[][2] = {
        {5, 5}, {10, 3}, {15, 5}, {17, 10}, {15, 15}, 
        {10, 17}, {5, 15}, {3, 10}
    };
    int num_star_points = sizeof(star_points) / sizeof(star_points[0]);
    
    // Fill the star with label 1
    for (int y = 3; y <= 17; y++) {
        for (int x = 3; x <= 17; x++) {
            // Simple point-in-polygon test (not accurate but good enough for this test)
            int inside = 0;
            for (int i = 0, j = num_star_points - 1; i < num_star_points; j = i++) {
                if (((star_points[i][1] > y) != (star_points[j][1] > y)) &&
                    (x < (star_points[j][0] - star_points[i][0]) * (y - star_points[i][1]) / 
                    (star_points[j][1] - star_points[i][1]) + star_points[i][0])) {
                    inside = !inside;
                }
            }
            if (inside) {
                pimg[y * nx + x] = 1;
            }
        }
    }
    
    // Draw shape 2 (label 2) - a simple rectangle with a hole
    for (int y = 20; y < 28; y++) {
        for (int x = 20; x < 28; x++) {
            if (y >= 22 && y <= 25 && x >= 22 && x <= 25) {
                // Hole
                continue;
            }
            pimg[y * nx + x] = 2;
        }
    }
    
    // Print the original image
    printf("Original image (label 1 = star-like, label 2 = rectangle with hole):\n");
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            if (pimg[y * nx + x] == 1) {
                printf("1 ");
            } else if (pimg[y * nx + x] == 2) {
                printf("2 ");
            } else {
                printf(". ");
            }
        }
        printf("\n");
    }
    
    // Compute the convex hull (8-connectivity)
    IMAGE* hull_img = _chull(test_img, 8);
    if (!hull_img) {
        fprintf(stderr, "Failed to compute hull\n");
        free_image(test_img);
        return;
    }
    
    // Print the hull
    print_hull_visual(hull_img, nx, ny);
    
    // Free memory
    free_image(test_img);
    free_image(hull_img);
    
    printf("Complex shapes test completed!\n");
}

// Performance test
void test_performance() {
    printf("\nRunning performance test...\n");
    
    // Create a large test image
    int nx = 500;
    int ny = 500;
    IMAGE* test_img = create_image(t_UINT32, nx, ny, 1);
    if (!test_img) {
        fprintf(stderr, "Failed to create test image\n");
        return;
    }
    
    // Fill image with background (0)
    UINT32* pimg = (UINT32*)GetImPtr(test_img);
    memset(pimg, 0, nx * ny * sizeof(UINT32));
    
    // Create multiple random shapes
    int num_shapes = 50;
    for (int shape = 1; shape <= num_shapes; shape++) {
        // Random center position
        int center_x = 50 + rand() % (nx - 100);
        int center_y = 50 + rand() % (ny - 100);
        
        // Random radius (irregular)
        int max_radius = 20 + rand() % 30;
        
        // Draw an irregular shape
        int num_vertices = 8 + rand() % 8;
        for (int angle = 0; angle < 360; angle += 5) {
            double rad = angle * 3.14159265 / 180.0;
            int radius = max_radius - rand() % (max_radius / 2);
            int x = center_x + (int)(radius * cos(rad));
            int y = center_y + (int)(radius * sin(rad));
            
            // Ensure within bounds
            if (x >= 0 && x < nx && y >= 0 && y < ny) {
                pimg[y * nx + x] = shape;
                
                // Draw some interior points
                int interior_x = (center_x + x) / 2;
                int interior_y = (center_y + y) / 2;
                if (interior_x >= 0 && interior_x < nx && interior_y >= 0 && interior_y < ny) {
                    pimg[interior_y * nx + interior_x] = shape;
                }
            }
        }
    }
    
    printf("Created test image with %d random shapes\n", num_shapes);
    
    // Measure time
    clock_t start = clock();
    
    // Compute the convex hull
    IMAGE* hull_img = _chull(test_img, 8);
    
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    
    if (!hull_img) {
        fprintf(stderr, "Failed to compute hull\n");
        free_image(test_img);
        return;
    }
    
    printf("Convex hull computed in %.3f seconds\n", elapsed);
    
    // Free memory
    free_image(test_img);
    free_image(hull_img);
}

// Main test function
int main() {
    printf("======= Testing C Convex Hull Implementation =======\n");
    
    // Test the chainHull_2D function
    test_chain_hull_2d();
    
    // Test the chull function on an image
    test_chull_on_image();
    
    // Test with more complex shapes
    test_real_world_cases();
    
    // Performance test
    test_performance();
    
    printf("All tests completed successfully!\n");
    return 0;
}
