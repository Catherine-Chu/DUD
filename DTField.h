//
// Created by 褚文杰 on 2019/11/19.
//

#ifndef DUD_DTFIELD_H
#define DUD_DTFIELD_H
# include <vector>

class DTField {
public:
    int gr_scale[2];
    std::vector<std::vector<int>> grids;
    std::vector<std::vector<int>> target_poses;
    std::vector<std::vector<double>> pixel_i;
    std::vector<std::vector<std::vector<double>>> pixel_gradient;
    DTField();
    void init_field(int w,int h);
    void reset_dt(int w, int h);
};


#endif //DUD_DTFIELD_H
