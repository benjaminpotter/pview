/// @file ccompass.h
/// @brief The single header file for the ccompass library.
///
/// @author Ben Potter
/// @date 19 May 2024
///
/// @see https://github.com/benjaminpotter/ccompass


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <errno.h>

// Math constants like M_PI are not part of the math.h standard.
// They are however provided by the implementation on my Linux machine.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
 
#ifndef CCOMPASS_H
#define CCOMPASS_H

struct cc_stokes {
    double i;
    double q;
    double u;
    double v;
};

/// 32bit color
struct cc_color {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
};

struct cc_color CC_WHITE = { 0xFF, 0xFF, 0xFF, 0xFF };
struct cc_color CC_BLACK = { 0x00, 0x00, 0x00, 0xFF };
struct cc_color CC_RED = { 0xFF, 0x00, 0x00, 0xFF };
struct cc_color CC_GREEN = { 0x00, 0xFF, 0x00, 0xFF };
struct cc_color CC_BLUE = { 0x00, 0x00, 0xFF, 0xFF };

// TODO Move these to global variables.
#define ACCUMULATOR_SIZE 36000
#define DOLP_THRESHOLD 0.07

const double CC_HOUGH_CROP_WIDTH = 512;
const double CC_HOUGH_BINARY_THRESHOLD = 0.02 * (M_PI / 180.0);


/// @brief Compute the Stokes vectors from a raw intensity matrix.
///
/// A division of focal plane (DoFP) camera can capture the instantaneous Stokes vector.
/// It has four linear polarizers arranged in a repeating tiling over the CCD sensor.
/// This function takes this intensity data as input and computes the Stokes vector.
///
/// Pattern of linear polarizers on the CCD.
///
/// +-----+-----+-----
/// | 090 | 135 | 090 
/// +-----+-----+-----
/// | 045 | 000 | 045
/// +-----+-----+-----
/// | 090 | 135 | ...
/// 
///
/// @param raw_image Array of intensity values read directly from the CCD sensor.
/// @param stokes_vectors Array of output Stokes vectors. Allocated by the caller.
/// @param w Width of input.
/// @param h Height of input.
/// @since 0.3
void cc_compute_stokes(unsigned char *raw_image, struct cc_stokes *stokes_vectors, int w, int h);


/// @brief Compute the angle of linear polarization from Stokes vectors.
/// 
/// @param stokes_vectors array of input Stokes vectors
/// @param aolps          array of output angles
/// @param w              width of input
/// @param h              height of input
/// @since                0.1
void cc_compute_aolp(struct cc_stokes *stokes_vectors, double *aolps, int w, int h);


/// @brief Compute the degree of linear polarization from Stokes vectors.
///
/// @param stokes_vectors array of input Stokes vectors
/// @param dolps          array of output fractions [0, 1]
/// @param w              width of input
/// @param h              height of input
/// @since                0.1
void cc_compute_dolp(struct cc_stokes *stokes_vectors, double *dolps, int w, int h);


/// This transformation is used to change the frame of reference
/// of a polarization image from instrument space to the solar
/// principal plane.
///
/// @brief Apply an in-place beta shift to Stokes vectors.
///
/// @param stokes_vectors array of input Stokes vectors
/// @param w              width of input
/// @param h              height of input
/// @since                0.1
void cc_transform_stokes(struct cc_stokes stokes_vectors[], int w, int h);


/// @brief Extract the solar azimuth using a 1D Hough transform on a 2D matrix of AoLP data
/// 
/// TODO
/// Requires further thought as to how the caller may set the control parameters
/// for their implementation. Should there be global context? Options passed to 
/// the function call? Etc.
///
/// @param angles  AoLP matrix
/// @param degress DoLP matrix
/// @param w       width of matrix
/// @param h       height of matrix
/// @param azimuth extracted solar azimuth
/// @since                0.1
void cc_hough_transform(double *angles, double *degrees, int w, int h, double *azimuth);


/// @brief Utility function for drawing a line restricted to the origin over a pixel matrix.
///
/// @param theta angle of the line from the x axis on range [-PI, PI]
/// @param pixels the image buffer to draw to
/// @param w width of matrix
/// @param h height of matrix
/// @since 0.1
void cc_draw_line(double theta, struct cc_color pixels[], int w, int h);


/// The input matrix is expected to have size elements. The dimensions of the
/// matrix are unimportant for the colour map computation. Each double is
/// converted to a 32 bit RGBA colour. 
///
/// TODO
/// There are some issues with how this function handles corner cases. The 
/// output should ideally replicate the _jet_ colour profile in Matlab.
///
/// @brief Utility function for converting a matrix of doubles into a colour image.
///
/// @param values input array of doubles
/// @param size   length of input
/// @param min    minimum value of input
/// @param max    maximum value of input
/// @param pixels output list of pixels 
/// @since                0.1
void cc_compute_cmap(double values[], int size, double min, double max, struct cc_color pixels[]);


/// @brief Utility function for generating a binary image from an AoLP matrix.
///
/// @see cc_hough_transform
/// This function aims to replicate the process that takes place in the Hough
/// transforms binary threshold phase. In the future (TODO) this function should
/// be more DRY, i.e., it should better reflect the actual process of the Hough
/// transform. Not suggested for use in debugging the Hough transform.
///
/// @param aolps input aolp matrix
/// @param dolps input dolp matrix
/// @param w width of aolp matrix
/// @param h height of aolp matrix
/// @param threshold value used to select pixels in the binary image
/// @param pixels output list of pixels 
/// @since 0.2
void cc_compute_binary_threshold(double aolps[], double dolps[], int w, int h, double threshold, struct cc_color pixels[]);


#ifdef CCOMPASS_IMPLEMENTATION

void cc_compute_stokes(unsigned char *raw_image, struct cc_stokes *stokes_vectors, int w, int h) {
    
    // Implemented in the most simple way imaginable.
    // One other option would be to apply a kernel to the image to create the stokes vector.
    
    // w and h refer to the size of the stokes vector matrix.

    for(int row = 0; row < h; ++row) {
        for(int col = 0; col < w; ++col) {
            double i000 = raw_image[(row*2+1) * 2 * w + (col*2+1)];
            double i045 = raw_image[(row*2+1) * 2 * w + (col*2+0)];
            double i090 = raw_image[(row*2+0) * 2 * w + (col*2+0)];
            double i135 = raw_image[(row*2+0) * 2 * w + (col*2+1)];

            // printf("%d %f\n", raw_image[(row*2+1) * 2 * w + (col*2+1)], i000);

            stokes_vectors[row * w + col].i = (i000 + i090); //  / (255.0 * 2);
            stokes_vectors[row * w + col].q = (i000 - i090); //  / 255.0;
            stokes_vectors[row * w + col].u = (i045 - i135); //  / 255.0;
            stokes_vectors[row * w + col].v = 0;
        }
    }
}

void cc_compute_aolp(struct cc_stokes *stokes_vectors, double *aolps, int w, int h) {

    for(int i = 0; i < w * h; ++i) {
        double q,u;
        q = stokes_vectors[i].q;
        u = stokes_vectors[i].u;

        aolps[i] = 0.5 * atan2(u, q);
    } 
    
}

void cc_compute_dolp(struct cc_stokes *stokes_vectors, double *dolps, int w, int h) {

    for(int i = 0; i < w * h; ++i) {

        if(stokes_vectors[i].i == 0.0) {
            dolps[i] = 0.0;
            continue;
        }

        double squared_magnitude = pow(stokes_vectors[i].q, 2) + pow(stokes_vectors[i].u, 2);
        dolps[i] = sqrt(squared_magnitude) / stokes_vectors[i].i;
    } 

}

void cc_compute_cmap(double values[], int size, double min, double max, struct cc_color pixels[]) {

    for(int i = 0; i < size; ++i) {
        double f = values[i];

        if(f < min)
            f = min;

        if(f > max)
            f = max;

        double df = max - min;
        double fr = 1.0, fg = 1.0, fb = 1.0;

        if(f < (min + 0.25 * df)) {
            fr = 0;
            fg = 4 * (f - min) / df;
        }

        else if(f < (min + 0.5 * df)) {
            fr = 0;
            fb = 1 + 4 * (min + 0.25 * df - f) / df;
        }
        
        else if(f < (min + 0.75 * df)) {
            fr = 4 * (f - min - 0.5 * df) / df;
            fb = 0;
        }

        else {
            fg = 1 + 4 * (min + 0.75 * df - f) / df;
            fb = 0;
        }

        pixels[i].r = fr * 255.0;
        pixels[i].g = fg * 255.0;
        pixels[i].b = fb * 255.0;
        pixels[i].a = 0xFF;
    }
    
}

void cc_transform_stokes(struct cc_stokes stokes_vectors[], int w, int h) {
    for(int i = 0; i < w * h; ++i) {
        // (x,y) position of the pixel in image space
        int x,y;

        x = i % w;
        x -= w/2;

        y = i / w;
        y -= h/2;
        // y *= -1;

        double beta;
        beta = 2.0 * atan2(y, x);
        
        double dx, dy;
        dx = cos(beta);
        dy = sin(beta);

        double q, u;
        q = stokes_vectors[i].q;
        u = stokes_vectors[i].u;

        stokes_vectors[i].q = q * dx + u * dy;
        stokes_vectors[i].u = u * dx - q * dy;
    }

}


double cc_linear_map(double x, double x_min, double x_max, double y_min, double y_max) {
    double x_normalized = (x - x_min) / (x_max - x_min);
    return x_normalized * (y_max - y_min) + y_min;
}

void cc_hough_transform(double *angles, double *degrees, int w, int h, double *azimuth) {

    const int mean_kernel_width = 8;

    // TODO cache the pixel positions in a lookup table rather
    // than computing them each time.

    int accumulator[ACCUMULATOR_SIZE];

    for(int i = 0; i < ACCUMULATOR_SIZE; ++i)
        accumulator[i] = 0;

    for(int i = 0; i < w * h; ++i) {

        // skip this pixel if the dolp is too low
        if(degrees[i] < DOLP_THRESHOLD)
            continue;

        // skip this pixel if it isn't in the threshold.
        if( fabs(angles[i] - M_PI_2) > CC_HOUGH_BINARY_THRESHOLD)
            continue;

        // (x,y) position of the pixel in image space.
        double x,y;
        x = i % w - 1024.0;
        y = -1 * (floor( i / w ) - 1024.0);

        //if(x < -CC_HOUGH_CROP_WIDTH || x > CC_HOUGH_CROP_WIDTH || y < -CC_HOUGH_CROP_WIDTH || y > CC_HOUGH_CROP_WIDTH)
        //     continue;
        
        double theta;
        theta = atan2(y, x);

        // "vote" for this theta in accumulator
        // theta is on the range [-M_PI, M_PI].
        int theta_index = cc_linear_map(theta, -M_PI, M_PI, 0, ACCUMULATOR_SIZE);
        accumulator[theta_index] += 1;
    }

    // apply a mean filter to the accumulator
    // this process is essentially a convolution
    // use "wrapping" based edge handling since angles are periodic over 2pi

    double filtered_accumulator[ACCUMULATOR_SIZE];
    for(int i = 0; i < ACCUMULATOR_SIZE; ++i) {
        double average = 0.0;

        // apply convolution, aka "convolve"
        for(int j = -(mean_kernel_width >> 1); j <= (mean_kernel_width >> 2); ++j) {
            int idx = i + j;

            // ensure wrapping
            if(idx < 0) { idx = ACCUMULATOR_SIZE + idx; }
            else if(idx >= ACCUMULATOR_SIZE) { idx = idx % ACCUMULATOR_SIZE; }

            average += accumulator[idx];
        }
        
        filtered_accumulator[i] = average / mean_kernel_width;
    }

    #if 0
    FILE *dump_file = fopen("accumulator.csv", "w"); 
    if(dump_file) {
        for(int i = 0; i < ACCUMULATOR_SIZE; ++i) {
            fprintf(
                dump_file, 
                "%0.3f,%d,%0.3f\n", 
                cc_linear_map(i, 0, ACCUMULATOR_SIZE, -180.0, 180.0),
                accumulator[i],
                filtered_accumulator[i]
            );         
        }

        fclose(dump_file);
    }
    #endif

    int azimuth_index = 0;
    for(int i = 0; i < ACCUMULATOR_SIZE; ++i) {
        if(filtered_accumulator[i] > filtered_accumulator[azimuth_index])
            azimuth_index = i;
    } 

    // convert index back to angle
    *azimuth = cc_linear_map(azimuth_index, 0, ACCUMULATOR_SIZE, -M_PI, M_PI);
}


void cc_draw_line(double theta, struct cc_color pixels[], int w, int h) {
    // implemented using Bresenham's Line Algorithm
    // https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
    
    struct cc_color LINE_COLOR = CC_WHITE;

    // by default dx is positive to the right
    // and dy is positive downward
    
    // clamp theta on range [-pi/2, pi/2]
    // theta = fmod(theta, M_PI);
    // TODO probably can just wrap the angle somehow
    if(theta > M_PI || theta < -M_PI)
        return;

    // is the slope > 1
    bool steep = fabs(theta) > 0.25 * M_PI && fabs(theta) < 0.75 * M_PI;
    
    if(steep) {
        double nx =1.0 + (1.0 / tan(theta));
        if(theta == M_PI_2)
            nx = 0.0;

        int x = ceil( (w / 2.0) * nx);

        int dx = (w - x) - x, dy = h;

        bool negative = false;
        if(dx < 0) {
            dx = -dx;
            negative = true;
        }

        int eps = 0;
        for(int y = 0; y < h; ++y) {
            // plot (x, y)
            pixels[ x + y*w ] = LINE_COLOR;

            eps += dx;

            if((eps << 1) >= dy ) {
                x += negative ? -1 : 1;
                eps -= dy;
            }
        }

    } else {
        // finding the initial y from theta
        double ny = 1.0 + tan(theta);
        int y = ceil( (h / 2.0) * ny );
        
        // generating a dx, dy pair from theta
        int dx = w, dy = (h - y) - y;

        // if dy is negative (implying negative slope) we must account
        // for this later
        bool negative = false;
        if(dy < 0) {
            dy = -dy;
            negative = true;
        }

        int eps = 0;
        for(int x = 0; x < w; ++x) {
            // plot (x, y)
            pixels[ x + y*w ] = LINE_COLOR;

            eps += dy;

            if((eps << 1) >= dx ) {
                y += negative ? -1 : 1;
                eps -= dx;
            }
        }
    }
}


void cc_compute_binary_threshold(double aolps[], double dolps[], int w, int h, double threshold, struct cc_color pixels[]) {
    for(int i = 0; i < w * h; ++i) {
        pixels[i] = CC_BLACK;

        // skip this pixel if the dolp is too low
        if(dolps[i] < DOLP_THRESHOLD)
            continue;

        // skip this pixel if it isn't in the threshold.
        if(fabs(aolps[i] - M_PI_2) > threshold)
            continue;

        // (x,y) position of the pixel in image space.
        double x,y;
        x = i % w - 1024.0;
        y = -1 * (floor( i / w ) - 1024.0);

        //if(x < -CC_HOUGH_CROP_WIDTH || x > CC_HOUGH_CROP_WIDTH || 
        //        y < -CC_HOUGH_CROP_WIDTH || y > CC_HOUGH_CROP_WIDTH)
        //     continue;
            
        pixels[i] = CC_WHITE; 
    }
}


#endif // CCOMPASS_IMPLEMENTATION 


#endif // CCOMPASS_H

