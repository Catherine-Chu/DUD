//
// Created by 褚文杰 on 2019/11/19.
//

#ifndef DUD_CLOUD_H
#define DUD_CLOUD_H
#include<vector>
#include<cmath>
#include<iostream>
#include "DTField.h"

class Cloud {
public:
    DTField dt;
    double measure_sc;
    int a_num;
    int ac_tar;
    std::vector<std::vector<int>> agent_poses;
    Cloud();
    Cloud(int a_num,int t_num, int w, int h, double sc);
    Cloud(int w, int h, double sc);
    void set_a_num(int a_num);
    void calculate_dt_gradient();
};


#endif //DUD_CLOUD_H
