//
// Created by 褚文杰 on 2019/11/19.
//

#include "DTField.h"

DTField::DTField() {
    gr_scale[0]=0;
    gr_scale[1]=0;
}

void DTField::init_field(int w, int h) {
    gr_scale[0] = w;
    gr_scale[1] = h;
//    shape_point_list.assign(s_point_list.begin(), s_point_list.end());
    for (int x = 0; x < w; x++) {
        grids.emplace_back();
        pixel_i.emplace_back();
        pixel_gradient.emplace_back();
        for (int y = 0; y < h; y++) {
            grids[x].push_back(0);
            pixel_i[x].push_back(0);
            pixel_gradient[x].push_back(std::vector<double>());
        }
    }
}