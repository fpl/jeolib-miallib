/***********************************************************************
Test case for the C++ implementation of the convex hull algorithm
***********************************************************************/

#include <iostream>
#include <vector>
#include <cassert>
#include <chrono>
#include <algorithm>
#include <cmath>

// Function to create a test image with a simple object
extern "C" {
#include "miallib.h"
}

#include "myhull.h"

// Visual representation of hull points
void print_hull_visual(IMAGE* hull_img, int nx, int ny) {
    UCHAR* pout = (UCHAR*)GetImPtr(hull_img);
    
    std::cout << "Visual representation of the hull (10 = hull points):" << std::endl;
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            int pos = y * nx + x;
            if (pout[pos] == 10) {
                std::cout << "X ";  // Hull point
            } else if (pout[pos] == 9) {
                std::cout << "O ";  // Border point
            } else if (pout[pos] > 0) {
                std::cout << ". ";  // Other marked point
            } else {
                std::cout << "  ";  // Empty
            }
        }
        std::cout << std::endl;
    }
}

// Test the chainHull_2D function directly
void test_chain_hull_2d() {
    std::cout << "Testing chainHull_2D function..." << std::endl;
    
    // Create a set of test points - a simple square with some interior points
    std::vector<intpair_t> points = {
        {1, 1}, {5, 1}, {5, 5}, {1, 5},  // Corners of a square
        {2, 2}, {3, 3}, {4, 2}, {2, 4}   // Interior points
    };
    
    // Print input points
    std::cout << "Input points:" << std::endl;
    for (const auto& p : points) {
        std::cout << "(" << p.a << ", " << p.b << ") ";
    }
    std::cout << std::endl;
    
    // Compute the convex hull
    std::vector<intpair_t> hull = chainHull_2D(points);
    
    // Print hull points
    std::cout << "Hull points:" << std::endl;
    for (const auto& p : hull) {
        std::cout << "(" << p.a << ", " << p.b << ") ";
    }
    std::cout << std::endl;
    
    // Verify the hull (should be the four corner points)
    assert(hull.size() == 4);
    bool has_point_1_1 = false;
    bool has_point_5_1 = false;
    bool has_point_5_5 = false;
    bool has_point_1_5 = false;
    
    for (const auto& p : hull) {
        if (p.a == 1 && p.b == 1) has_point_1_1 = true;
        if (p.a == 5 && p.b == 1) has_point_5_1 = true;
        if (p.a == 5 && p.b == 5) has_point_5_5 = true;
        if (p.a == 1 && p.b == 5) has_point_1_5 = true;
    }
    
    assert(has_point_1_1 && has_point_5_1 && has_point_5_5 && has_point_1_5);
    std::cout << "chainHull_2D test passed!" << std::endl;
}

// Test the chull function on an image
void test_chull_on_image() {
    std::cout << "\nTesting chull function on an image..." << std::endl;
    
    // Create a test image (20x20 with a simple shape)
    int nx = 20;
    int ny = 20;
    IMAGE* test_img = create_image(t_UINT32, nx, ny, 1);
    if (!test_img) {
        std::cerr << "Failed to create test image" << std::endl;
        return;
    }
    
    // Fill image with background (0)
    UINT32* pimg = (UINT32*)GetImPtr(test_img);
    for (int i = 0; i < nx * ny; i++) {
        pimg[i] = 0;
    }
    
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
    std::cout << "Original image (label 1 = L shape):" << std::endl;
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            std::cout << (pimg[y * nx + x] > 0 ? "# " : ". ");
        }
        std::cout << std::endl;
    }
    
    // Compute the convex hull (8-connectivity)
    IMAGE* hull_img = chull(test_img, 8);
    if (!hull_img) {
        std::cerr << "Failed to compute hull" << std::endl;
        free_image(test_img);
        return;
    }
    
    // Print the hull
    print_hull_visual(hull_img, nx, ny);
    
    // Free memory
    free_image(test_img);
    free_image(hull_img);
    
    std::cout << "chull test completed!" << std::endl;
}

// Test with real-world scenarios
void test_real_world_cases() {
    std::cout << "\nTesting with more complex shapes..." << std::endl;
    
    // Create a test image (30x30 with multiple shapes)
    int nx = 30;
    int ny = 30;
    IMAGE* test_img = create_image(t_UINT32, nx, ny, 1);
    if (!test_img) {
        std::cerr << "Failed to create test image" << std::endl;
        return;
    }
    
    // Fill image with background (0)
    UINT32* pimg = (UINT32*)GetImPtr(test_img);
    for (int i = 0; i < nx * ny; i++) {
        pimg[i] = 0;
    }
    
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
    std::cout << "Original image (label 1 = star-like, label 2 = rectangle with hole):" << std::endl;
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            if (pimg[y * nx + x] == 1) {
                std::cout << "1 ";
            } else if (pimg[y * nx + x] == 2) {
                std::cout << "2 ";
            } else {
                std::cout << ". ";
            }
        }
        std::cout << std::endl;
    }
    
    // Compute the convex hull (8-connectivity)
    IMAGE* hull_img = chull(test_img, 8);
    if (!hull_img) {
        std::cerr << "Failed to compute hull" << std::endl;
        free_image(test_img);
        return;
    }
    
    // Print the hull
    print_hull_visual(hull_img, nx, ny);
    
    // Free memory
    free_image(test_img);
    free_image(hull_img);
    
    std::cout << "Complex shapes test completed!" << std::endl;
}

// Performance test
void test_performance() {
    std::cout << "\nRunning performance test..." << std::endl;
    
    // Create a large test image
    int nx = 500;
    int ny = 500;
    IMAGE* test_img = create_image(t_UINT32, nx, ny, 1);
    if (!test_img) {
        std::cerr << "Failed to create test image" << std::endl;
        return;
    }
    
    // Fill image with background (0)
    UINT32* pimg = (UINT32*)GetImPtr(test_img);
    std::fill(pimg, pimg + nx * ny, 0);
    
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
    
    std::cout << "Created test image with " << num_shapes << " random shapes" << std::endl;
    
    // Measure time
    auto start = std::chrono::high_resolution_clock::now();
    
    // Compute the convex hull
    IMAGE* hull_img = chull(test_img, 8);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    
    if (!hull_img) {
        std::cerr << "Failed to compute hull" << std::endl;
        free_image(test_img);
        return;
    }
    
    std::cout << "Convex hull computed in " << elapsed.count() << " seconds" << std::endl;
    
    // Free memory
    free_image(test_img);
    free_image(hull_img);
}

// Main test function
int main() {
    std::cout << "======= Testing C++ Convex Hull Implementation =======" << std::endl;
    
    // Test the chainHull_2D function
    test_chain_hull_2d();
    
    // Test the chull function on an image
    test_chull_on_image();
    
    // Test with more complex shapes
    test_real_world_cases();
    
    // Performance test
    test_performance();
    
    std::cout << "All tests completed successfully!" << std::endl;
    return 0;
}
