#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <algorithm>
#include <thread>
#include "Agent.h"

using namespace std;
#define THREAD_NUM 1
#define PI acos(-1)
int a_num = 274/5;
int t_num = 274;
const int width = 40;
const int height = 40;
double measure_scale = 1;
int v_max = 1;
double force_coff = 10;
Cloud center = Cloud(a_num, t_num, width, height,measure_scale);
vector<Agent> swarm(a_num);

struct ThreadArg {
    int thread_id;
    int s_id;
    int e_id;

    ThreadArg() {
        thread_id = -1;
        s_id = -1;
        e_id = -1;
    };
};

void test_func(int t_id, int s_id, int e_id) {
    for (int i = s_id; i <= e_id; i++) {
        cout << "Thread " << t_id << ": agent" << i << endl;
    }
}
void chooseActions(int t_id, int s_id, int e_id) {
    for (int i = s_id; i <= e_id; i++) {
        swarm[i].choose_action();
    }
}
void takeActions(int t_id, int s_id, int e_id) {
    for (int i = s_id; i <= e_id; i++) {
        swarm[i].take_action();
    }
}

Agent::Agent(int id, int px, int py, int r) {
    index = id;
    pos_x = px;
    pos_y = py;
    sense_r = r;
    last_action[0]=0;
    last_action[1]=0;
    repulse[0]=0.0;
    repulse[1]=0.0;
}

Agent::Agent() {
    index = -1;
    sense_r = 1;
    last_action[0]=0;
    last_action[1]=0;
    repulse[0]=0.0;
    repulse[1]=0.0;
}

void Agent::set_config(int id, int px, int py, int r) {
    index = id;
    pos_x = px;
    pos_y = py;
    sense_r = r;
    last_action[0]=0;
    last_action[1]=0;
    repulse[0]=0.0;
    repulse[1]=0.0;
}

bool Agent::get_local_info() {
    repulse[0] = 0.0;
    repulse[1] = 0.0;
    double alpha = 1;
    bool has_neighbor = false;
    for (int i = pos_x - sense_r; i < pos_x + sense_r+1; i++) {
        for (int j = pos_y - sense_r; j < pos_y + sense_r+1; j++) {
            if (i >= 0 && i < center.dt.gr_scale[0] && j >= 0 && j < center.dt.gr_scale[1]) {
//                if(i==pos_x && j==pos_y && center.agent_poses[pos_x][pos_y]!=index){
//                    cout<<index<<" conflict2 "<<center.agent_poses[pos_x][pos_y]<<endl;
//                }
                if (center.agent_poses[i][j] >= 0 && !(i==pos_x && j==pos_y)) {
                    double dis = measure_scale*sqrt(pow(i-pos_x,2)+pow(j-pos_y,2));
                    if(dis<=sense_r) {
                        double coff = alpha * ((1 / (sense_r*measure_scale)) - 1 / dis) * (1 / pow(dis, 2));
                        repulse[0] += coff * measure_scale*(i - pos_x);
                        repulse[1] += coff * measure_scale*(j - pos_y);
                        has_neighbor = true;
                    }
                }
            }
        }
    }
    return has_neighbor;
}

void Agent::choose_action() {
    bool has_neighbor = get_local_info();
    double x_force = repulse[0]-center.dt.pixel_gradient[pos_x][pos_y][0];
    double y_force = repulse[1]-center.dt.pixel_gradient[pos_x][pos_y][1];
//    double x_force = -center.dt.pixel_gradient[pos_x][pos_y][0];
//    double y_force = -center.dt.pixel_gradient[pos_x][pos_y][1];
//    cout<<index<<':'<<pos_x<<','<<pos_y<<' '<<x_force<<','<<y_force<<endl;
//    int step = round(fmod(force_coff*sqrt(pow(x_force,2)+pow(y_force,2)),v_max));
    int step=1;
    next_x=0;
    next_y=0;
    if(!(x_force==0 && y_force==0)) {
        double angle = 0;
        if (x_force != 0) {
            angle = atan(y_force / x_force);
            if (x_force < 0) {
                if (y_force > 0) {
                    angle = angle + PI;
                }else if(y_force<0){
                    angle = angle - PI;
                }else{
                    angle = -PI;
                }
            }
        } else if (y_force > 0) {
            angle = PI / 2;
        } else if (y_force < 0) {
            angle = -PI / 2;
        }
        if(angle>-PI/4 && angle<PI/4){
            next_x=pos_x+step;
            next_y=pos_y;
        }else if(angle==PI/4){
            next_x=pos_x+step;
            next_y=pos_y+step;
        }else if(angle>PI/4 && angle<3*PI/4){
            next_x=pos_x;
            next_y=pos_y+step;
        }else if(angle==3*PI/4){
            next_x=pos_x-step;
            next_y=pos_y+step;
        }else if(angle>3*PI/4 || angle<-3*PI/4){
            next_x=pos_x-step;
            next_y=pos_y;
        }else if(angle==-3*PI/4){
            next_x=pos_x-step;
            next_y=pos_y-step;
        }else if(angle<-PI/4 && angle>-3*PI/4){
            next_x=pos_x;
            next_y=pos_y-step;
        }else{
            next_x=pos_x+step;
            next_y=pos_y-step;
        }
    }else{
        if(!has_neighbor && repulse[0]==0 && repulse[1]==0) {
            //不受力的时候保持原本的运动趋势
            next_x = pos_x + last_action[0];
            next_y = pos_y + last_action[1];
        }else if(has_neighbor){
            //斥力引力平衡(所有斥力之间平衡或者是总斥力与总引力平衡,总之一定有斥力有neighbor)
            next_x = pos_x;
            next_y = pos_y;
        }
//        next_x = pos_x;
//        next_y = pos_y;
    }
    if(!(next_x>=0 && next_x<center.dt.gr_scale[0] && next_y>=0 && next_y<center.dt.gr_scale[1])) {
        next_x=pos_x;
        next_y=pos_y;
    }
}

// if multiple threads, before take actions, center.agent_poses must be cleared
void Agent::take_action() {
    if(center.agent_poses[next_x][next_y]==-1){
        last_action[0]=next_x-pos_x;
        last_action[1]=next_y-pos_y;
        pos_x=next_x;
        pos_y=next_y;
        center.agent_poses[pos_x][pos_y] = index;
    }else{
        last_action[0]=0;
        last_action[1]=0;
        center.agent_poses[pos_x][pos_y] = index;
    }
}

int main() {
    clock_t startT, endT;
    vector<double> dec_times;
    fstream infile;
    ofstream outfile("/Users/chuwenjie/CLionProjects/DUD/agent_poses.txt", ios::app);
    char s[width + 1];
    infile.open("/Users/chuwenjie/CLionProjects/InitSettingGenerator/grids.txt", ios::in);
    if (!infile) {
        cout << "open failed" << endl;
        exit(1);
    }
    int i = 0;
    vector<int> valid_l;
    outfile << "arguments: " << width << ' ' << height << ' ' << a_num << ' ' << THREAD_NUM << endl;
    while (!infile.eof() and i < width) {
        int j = 0;
        while (j < height) {
            infile >> center.dt.grids[i][j];
            if(center.dt.grids[i][j]==1){
                vector<int> pos = {i,j};
                center.dt.target_poses.push_back(pos);
            }
            valid_l.push_back(i * height + j);
            j++;
        }
        i++;
    }
    infile.close();
    int max_size = valid_l.size();
    srand((unsigned) time(NULL));
    for (int r = 0; r < a_num; r++) {
        int rand_p = rand() % max_size;
        int px = (int) valid_l[rand_p] / height;
        int py = (int) valid_l[rand_p] % height;
        center.agent_poses[px][py] = r;
        swarm[r].set_config(r,px,py,2);
        int tmp = valid_l[max_size - 1];
        valid_l[max_size - 1] = valid_l[rand_p];
        valid_l[rand_p] = tmp;
        max_size--;
    }
    vector<thread> threads;
    default_random_engine generator{random_device{}()};

    int terminal = 0;
    center.calculate_dt_gradient();
//    for(int i=0;i<width;i++){
//        for(int j=0;j<height;j++){
//            cout<<center.dt.pixel_i[i][j]<<' ';
//        }
//        cout<<endl;
//    }
//    for(int i=0;i<width;i++){
//        for(int j=0;j<height;j++){
//            cout<<center.dt.pixel_gradient[i][j][0]<<','<<center.dt.pixel_gradient[i][j][1]<<' ';
//        }
//        cout<<endl;
//    }
    center.ac_tar=0;
    for(int k=0;k<a_num;k++){
        if(center.dt.grids[swarm[k].pos_x][swarm[k].pos_y]==0){
            center.ac_tar+=1;
        }
    }
    outfile << "agent positions:" << endl;
    for (int k = 0; k < a_num; k++) {
        if (k < a_num - 1) {
            outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
        } else {
            outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
        }
    }
    outfile << endl;

    while (center.ac_tar>0 && terminal<2000) {
        cout << center.ac_tar << endl;
//        for (int i = 0; i < width; i++) {
//            for (int j = 0; j < height; j++) {
//                cout << center.agent_poses[i][j] << ' ';
//            }
//            cout << endl;
//        }
//        cout << endl;
        startT = clock();
        terminal += 1;

        int left = int(a_num % THREAD_NUM);
        int alloc = 0;
        int s_ids[THREAD_NUM];
        int e_ids[THREAD_NUM];
        srand(time(NULL));
        for (int k = 0; k < THREAD_NUM; k++) {
            s_ids[k] = -1, e_ids[k] = -1;
            if (left > alloc) {
                s_ids[k] = k * (int(a_num / THREAD_NUM) + 1);
                e_ids[k] = s_ids[k] + int(a_num / THREAD_NUM);
                alloc++;
            } else {
                s_ids[k] = alloc * (int(a_num / THREAD_NUM) + 1) + (k - alloc) * int(a_num / THREAD_NUM);
                e_ids[k] = s_ids[k] + int(a_num / THREAD_NUM) - 1;
            }
        }
        for (int k = 0; k < THREAD_NUM; k++) {
            threads.emplace_back(chooseActions, k, s_ids[k], e_ids[k]);
        }
        // 等待其他线程join
        for (int k = 0; k < THREAD_NUM; k++) {
            threads[k].join();
        }
        threads.clear();

        vector<int> line(height, -1);
        vector<vector<int>> array(width);
        for (int k = 0; k < array.size(); k++) {
            array[k].assign(line.begin(), line.end());
        }
        center.agent_poses.swap(array);

//        for (int k = 0; k < THREAD_NUM; k++) {
//            threads.emplace_back(takeActions, k, s_ids[k], e_ids[k]);
//        }
//        // 等待其他线程join
//        for (int k = 0; k < THREAD_NUM; k++) {
//            threads[k].join();
//        }
//        threads.clear();
        for(int k=0;k<a_num;k++){
            swarm[k].take_action();
        }
        center.ac_tar=0;
        for(int k=0;k<a_num;k++){
            if(center.dt.grids[swarm[k].pos_x][swarm[k].pos_y]==0){
                center.ac_tar+=1;
            }
        }
        endT = clock();
        dec_times.push_back((double) (endT - startT));

        outfile << "agent positions:" << endl;
        for (int k = 0; k < a_num; k++) {
            if (k < a_num - 1) {
                outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
            } else {
                outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
            }
        }
        outfile << endl;

    }
    double avg_t = 0;
    for (int k = 0; k < dec_times.size(); k++) {
        avg_t += dec_times[k];
    }

    cout << "The average decision time for each iteration is: " << avg_t / (dec_times.size() * CLOCKS_PER_SEC) << "s"
         << endl;
    cout << "Main: program exiting after " << terminal << " steps." << endl;
    outfile.close();
    return 0;
}