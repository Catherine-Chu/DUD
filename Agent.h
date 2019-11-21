//
// Created by 褚文杰 on 2019/11/19.
//

#ifndef DUD_AGENT_H
#define DUD_AGENT_H
#include<unordered_map>
#include<tuple>
#include<cstdlib>
#include<ctime>
#include "Cloud.h"
class Agent {
public:
    int index;
    int pos_x;
    int pos_y;
    int sense_r;
    double repulse[2];
    int last_action[2];
    int next_x;
    int next_y;
    Agent();
    Agent(int id, int px, int py, int r=1);
    void set_config(int id, int px, int py, int r=1);
    bool get_local_info();
    void choose_action();
    void take_action();
};
#endif //DUD_AGENT_H
