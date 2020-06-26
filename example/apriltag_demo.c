/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

// Invoke:
//
// tagtest [options] input.pnm

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_int(getopt, 'a', "hamming", "1", "Detect tags with up to this many bit errors.");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    //add cam params
    getopt_add_int(getopt, 'w', "width", "640", "The image width");
    getopt_add_int(getopt, 'l', "height", "480", "The image height");

    getopt_add_double(getopt, 'm', "fx", "320.83628590652455", "The image fx");
    getopt_add_double(getopt, 'n', "fy", "323.04325776720174", "The image fy");
    getopt_add_double(getopt, 'v', "cx", "320.2332167518086", "The image cx");
    getopt_add_double(getopt, 'c', "cy", "234.12811257055012", "The image cy");

    getopt_add_double(getopt, '1', "k1", "-0.24241406656348882", "The image k1");
    getopt_add_double(getopt, '2', "k2", "0.0402747578682183", "The image k2");
    getopt_add_double(getopt, '3', "p1", "-5.477653022258039e-06", "The image p1");
    getopt_add_double(getopt, '4', "p2", "-0.0005012637588869646", "The image p2");
    getopt_add_double(getopt, '5', "k3", "0.0", "The image k3");

    getopt_add_double(getopt, 'r', "p11", "211.56044006347656", "The image p11");
    getopt_add_double(getopt, '9', "p12", "0.0", "The image p12");
    getopt_add_double(getopt, ',', "p13", "318.87211753874", "The image p13");
    getopt_add_double(getopt, '|', "p21", "0.0", "The image p21");
    getopt_add_double(getopt, 'u', "p22", "268.90362548828125", "The image p22");
    getopt_add_double(getopt, 'e', "p23", "231.14138426406498", "The image p23");
    getopt_add_double(getopt, 'g', "p31", "0.0", "The image p31");
    getopt_add_double(getopt, 'p', "p32", "0.0", "The image p32");
    getopt_add_double(getopt, 'z', "p33", "1.0", "The image p33");


    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);

    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, getopt_get_int(getopt, "hamming"));
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    // Fix the rotation of our homography to properly orient the tag

    //cam param for rectification
    int imgwidth = getopt_get_int(getopt, "width");
    int imgheight = getopt_get_int(getopt, "height");

    double fx = getopt_get_double(getopt, "fx");
    double fy = getopt_get_double(getopt, "fy");
    double cx = getopt_get_double(getopt, "cx");
    double cy = getopt_get_double(getopt, "cy");

    double k1 = getopt_get_double(getopt, "k1");
    double k2 = getopt_get_double(getopt, "k2");
    double p1 = getopt_get_double(getopt, "p1");
    double p2 = getopt_get_double(getopt, "p2");
    double k3 = getopt_get_double(getopt, "k3");

    double p11 = getopt_get_double(getopt, "p11");
    double p12 = getopt_get_double(getopt, "p12");
    double p13 = getopt_get_double(getopt, "p13");
    double p21 = getopt_get_double(getopt, "p21");
    double p22 = getopt_get_double(getopt, "p22");
    double p23 = getopt_get_double(getopt, "p23");
    double p31 = getopt_get_double(getopt, "p31");
    double p32 = getopt_get_double(getopt, "p32");
    double p33 = getopt_get_double(getopt, "p33");

    apriltag_camera_info_t cinfo = {
        .width = imgwidth,
        .height = imgheight,
        .K = {
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        },
        .P = {
            p11, p12, p13, 0.0,
            p21, p22, p23, 0.0,
            p31, p32, p33, 0.0
        },
        .D = {
            k1, k2,
            p1, p2,
            k3
        }
    };


    int quiet = getopt_get_bool(getopt, "quiet");

    int maxiters = getopt_get_int(getopt, "iters");

    const int hamm_hist_max = 10;

    for (int iter = 0; iter < maxiters; iter++) {

        int total_quads = 0;
        int total_hamm_hist[hamm_hist_max];
        memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
        double total_time = 0;

        if (maxiters > 1)
            printf("iter %d / %d\n", iter + 1, maxiters);

        for (int input = 0; input < zarray_size(inputs); input++) {

            int hamm_hist[hamm_hist_max];
            memset(hamm_hist, 0, sizeof(hamm_hist));

            char *path;
            zarray_get(inputs, input, &path);
            if (!quiet)
                printf("loading %s\n", path);
            else
                printf("%20s ", path);

            image_u8_t *im = NULL;
            if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM") ||
                str_ends_with(path, "pgm") || str_ends_with(path, "PGM"))
                im = image_u8_create_from_pnm(path);
            else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
                if (str_ends_with(path, "raw.jpg")) {
                    init_rectification_maps(td, cinfo);
                }
                int err = 0;
                pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
                if (pjpeg == NULL) {
                    printf("pjpeg error %d\n", err);
                    continue;
                }

                if (1) {
                    im = pjpeg_to_u8_baseline(pjpeg);
                } else {
                    printf("illumination invariant\n");

                    image_u8x3_t *imc =  pjpeg_to_u8x3_baseline(pjpeg);

                    im = image_u8_create(imc->width, imc->height);

                    for (int y = 0; y < imc->height; y++) {
                        for (int x = 0; x < imc->width; x++) {
                            double r = imc->buf[y*imc->stride + 3*x + 0] / 255.0;
                            double g = imc->buf[y*imc->stride + 3*x + 1] / 255.0;
                            double b = imc->buf[y*imc->stride + 3*x + 2] / 255.0;

                            double alpha = 0.42;
                            double v = 0.5 + log(g) - alpha*log(b) - (1-alpha)*log(r);
                            int iv = v * 255;
                            if (iv < 0)
                                iv = 0;
                            if (iv > 255)
                                iv = 255;

                            im->buf[y*im->stride + x] = iv;
                        }
                    }
                    image_u8x3_destroy(imc);
                    if (td->debug)
                        image_u8_write_pnm(im, "debug_invariant.pnm");
                }

                pjpeg_destroy(pjpeg);
            }

            if (im == NULL) {
                printf("couldn't load %s\n", path);
                continue;
            }

            zarray_t *detections = apriltag_detector_detect(td, im);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                if (!quiet)
                    printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                           i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);

                hamm_hist[det->hamming]++;
                total_hamm_hist[det->hamming]++;
            }

            apriltag_detections_destroy(detections);

            if (!quiet) {
                timeprofile_display(td->tp);
            }

            total_quads += td->nquads;

            if (!quiet)
                printf("hamm ");

            for (int i = 0; i < hamm_hist_max; i++)
                printf("%5d ", hamm_hist[i]);

            double t =  timeprofile_total_utime(td->tp) / 1.0E3;
            total_time += t;
            printf("%12.3f ", t);
            printf("%5d", td->nquads);

            printf("\n");

            image_u8_destroy(im);
        }


        printf("Summary\n");

        printf("hamm ");

        for (int i = 0; i < hamm_hist_max; i++)
            printf("%5d ", total_hamm_hist[i]);
        printf("%12.3f ", total_time);
        printf("%5d", total_quads);
        printf("\n");

    }

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }

    getopt_destroy(getopt);

    return 0;
}
