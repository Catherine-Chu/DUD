//
// Created by 褚文杰 on 2019/11/19.
//

#include "Cloud.h"

using namespace std;

Cloud::Cloud() = default;

Cloud::Cloud(int a_num, int t_num, int w, int h, double sc) {
    dt.init_field(w, h);
    this->a_num = a_num;
    ac_tar = 0;
    for (int x = 0; x < w; x++) {
        agent_poses.emplace_back();
        for (int y = 0; y < h; y++) {
            agent_poses[x].push_back(-1);
        }
    }
    measure_sc=sc;
}

void Cloud::calculate_dt_gradient() {
    //在所有iteration开始之前一次性计算
    for(int i=0;i<dt.gr_scale[0];i++){
        for(int j=0;j<dt.gr_scale[1];j++){
            if(dt.grids[i][j]==1){
                dt.pixel_i[i][j]=0;
            }else{
                dt.pixel_i[i][j]=100000000;
                for(int t=0;t<dt.target_poses.size();t++){
                    double dis = measure_sc*sqrt(pow(dt.target_poses[t][0]-i,2)+pow(dt.target_poses[t][1]-j,2));
                    if(dt.pixel_i[i][j]>dis){
                        dt.pixel_i[i][j]=dis;
                    }
                }
            }
        }
    }
    for(int i=0;i<dt.gr_scale[0];i++){
        for(int j=0;j<dt.gr_scale[1];j++){
            double x_g=0;
//            if(i>0 && i<dt.gr_scale[0]-1)
//                x_g = (dt.pixel_i[i+1][j]-dt.pixel_i[i-1][j])/2;
//            else if(i==0)
//                x_g = dt.pixel_i[i+1][j] - dt.pixel_i[i][j];
//            else
//                x_g = dt.pixel_i[i][j] - dt.pixel_i[i-1][j];
//            dt.pixel_gradient[i][j].push_back(x_g);
//            double y_g = 0;
//            if(j>0 && j<dt.gr_scale[1]-1)
//                y_g=(dt.pixel_i[i][j+1]-dt.pixel_i[i][j-1])/2;
//            else if (j==0)
//                y_g=dt.pixel_i[i][j+1]-dt.pixel_i[i][j];
//            else
//                y_g=dt.pixel_i[i][j]-dt.pixel_i[i][j-1];
//            dt.pixel_gradient[i][j].push_back(y_g);
            if(i<dt.gr_scale[0]-1)
                x_g = (dt.pixel_i[i+1][j]-dt.pixel_i[i][j])/measure_sc;
            else
                x_g = (dt.pixel_i[i][j] - dt.pixel_i[i-1][j])/measure_sc;
            dt.pixel_gradient[i][j].push_back(x_g);
            double y_g = 0;
            if(j<dt.gr_scale[1]-1)
                y_g=(dt.pixel_i[i][j+1]-dt.pixel_i[i][j])/measure_sc;
            else
                y_g=(dt.pixel_i[i][j]-dt.pixel_i[i][j-1])/measure_sc;
            dt.pixel_gradient[i][j].push_back(y_g);
        }
    }
}


