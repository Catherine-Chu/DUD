#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string>
#include <cmath>
#include <random>
#include <algorithm>
#include <thread>
#include "Agent.h"

using namespace std;
#define TEMP_THREAD_NUM 1
#define PI acos(-1)
const string RUN_ENV = "MAC";
//int a_num = 274/5;
//int t_num = 274;
const int width = 40;
const int height = 40;
double measure_scale = 1;
int v_max = 1;
double force_coff = 10;
//Cloud center = Cloud(a_num, t_num, width, height,measure_scale);
//vector<Agent> swarm(a_num);
Cloud center;
vector<Agent> swarm;
vector<vector<vector<int>>> agent_init_pos_in_shapes; //7*agent_num*2
enum Agent_num_mode {
    less_a = -1, equal_a = 0, more_a = 1
};
Agent_num_mode a_mode = more_a;

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
    center.agent_poses[pos_x][pos_y]=id;
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
                    if(dis<=sqrt(2)) {
                        double coff = alpha * ((1 / (sense_r*measure_scale)) - 1 / dis) * (1 / pow(dis, 2));
                        repulse[0] += coff * measure_scale*(i - pos_x);
                        repulse[1] += coff * measure_scale*(j - pos_y);
                        has_neighbor = true;
                    }
//                    double dis = measure_scale*(max(abs(i-pos_x),abs(j-pos_y))+1);
//                    if(dis<=2) {
//                        double coff = alpha * ((1 / ((sense_r+1)*measure_scale)) - 1 / dis) * (1 / pow(dis, 2));
//                        repulse[0] += coff * measure_scale*(i - pos_x);
//                        repulse[1] += coff * measure_scale*(j - pos_y);
//                        has_neighbor = true;
//                    }
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
//    if(center.agent_poses[next_x][next_y]==-1){
//        last_action[0]=next_x-pos_x;
//        last_action[1]=next_y-pos_y;
//        pos_x=next_x;
//        pos_y=next_y;
//        center.agent_poses[pos_x][pos_y] = index;
//    }else{
//        last_action[0]=0;
//        last_action[1]=0;
//        center.agent_poses[pos_x][pos_y] = index;
//    }
    if(center.agent_poses[next_x][next_y]==-1){
        center.agent_poses[pos_x][pos_y]=-1;
        last_action[0]=next_x-pos_x;
        last_action[1]=next_y-pos_y;
        pos_x=next_x;
        pos_y=next_y;
        center.agent_poses[pos_x][pos_y] = index;
    }else{
        last_action[0]=0;
        last_action[1]=0;
    }
}

double getMold(const vector<vector<int>>& vec){   //求向量的模长
    int n = vec.size();
    double sum = 0.0;
    for (int i = 0; i<n; ++i) {
        int m = vec[i].size();
        for(int j=0; j<m; ++j) {
            sum += vec[i][j] * vec[i][j];
        }
    }
    return sqrt(sum);
}

double getSimilarity(const vector<vector<int>>& lhs, const vector<vector<int>>& rhs){
    int n = lhs.size();
    if(n == rhs.size()) {
        double tmp = 0.0;  //内积
        for (int i = 0; i < n; ++i){
            int m = lhs[i].size();
            if(m == rhs[i].size()) {
                for(int j=0; j<m; ++j){
                    tmp += lhs[i][j] * rhs[i][j];
                }
            }else{
                return -1;
            }
        }
        return tmp / (getMold(lhs) * getMold(rhs));
    }else{
        return -1;
    }
}

bool initialize_no_seed_agent_positions(int f, int min_i, int min_j, int shape_agent_num) {
    int cnt = 4;
    bool is_enough = false;
    if (min_j - 3 < 4 or min_i < 4 or min_i >= width - 4) {
        cout << "Too narrow space for edge following." << endl;
        return is_enough;
    }

    for (int j = min_j - 3; j >= 4; j--) {
//        swarm[cnt].set_config(min_i,j,2*sqrt(2));
        agent_init_pos_in_shapes[f].push_back({min_i, j});
        cnt++;
        if (cnt == shape_agent_num) {
            is_enough = true;
            break;
        }
    }
    int max_height = int(0.6 * height);
    vector<vector<int>> incre_options;
    if (not is_enough) {
        for (int i = min_i + 1; i < width - 4; i++) {
            for (int j = 4; j <= max_height; j++) {
                if (j > min_j - 3) {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 2 && px<width-1; px++) {
                        for (int py = j - 2; py <= j + 2 && py<height-1; py++) {
//                            cout<<px<<" "<<py<<endl;
                            if (center.dt.grids[px][py] == 0) {
                                is_valid = is_valid && true;
                            } else {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid) {
                            break;
                        }
                    }
                    if (is_valid) {
//                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    } else {
                        break;
                    }
                } else {
//                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num) {
                    is_enough = true;
                    break;
                }
                if (j == max_height) {
                    incre_options.push_back({i, j});
                }
            }
            if (is_enough) {
                break;
            }
        }
    }
    if (not is_enough) {
        for (int i = min_i - 1; i >= 4; i--) {
            for (int j = 4; j <= max_height; j++) {
                if (j > min_j - 3) {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 3 && px<width-1; px++) {
                        for (int py = j - 2; py <= j + 2 &&py<height-1; py++) {
//                            cout<<px<<" "<<py<<endl;
                            if (center.dt.grids[px][py] == 0) {
                                is_valid = is_valid && true;
                            } else {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid) {
                            break;
                        }
                    }
                    if (is_valid) {
//                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    } else {
                        break;
                    }
                } else {
//                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num) {
                    is_enough = true;
                    break;
                }
                if (j == max_height) {
                    incre_options.push_back({i, j});
                }
            }
            if (is_enough) {
                break;
            }
        }
    }
    if (not is_enough) {
        for (int o = 0; o < incre_options.size(); o++) {
            int i = incre_options[o][0];
            for (int j = incre_options[o][1] + 1; j < height-2; j++) {
                if (j > min_j - 3) {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 2 && px<width-1; px++) {
                        for (int py = j - 2; py <= j + 2 && py<height-1; py++) {
//                            cout<<px<<" "<<py<<endl;
                            if (center.dt.grids[px][py] == 0) {
                                is_valid = is_valid && true;
                            } else {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid) {
                            break;
                        }
                    }
                    if (is_valid) {
//                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    } else {
                        break;
                    }
                } else {
//                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num) {
                    is_enough = true;
                    break;
                }
            }
            if (is_enough) {
                break;
            }
        }
    }

    if (not is_enough) {
        cout << "There isn't enough space for initialization." << endl;

    }
    return is_enough;
}


int main() {
    string dictionary, out_dict;
    if (RUN_ENV == "WIN") {
        dictionary =
                "D:\\projects\\CLionProjects\\InitSettingGenerator\\tmp_edge_display\\" + to_string(width) + '_' + to_string(height);
        out_dict = "D:\\projects\\CLionProjects\\DUD\\exp";
    } else if (RUN_ENV == "MAC") {
        dictionary =
                "/Users/chuwenjie/CLionProjects/InitSettingGenerator/tmp_edge_display/" + to_string(width) + '*' + to_string(height);
        out_dict = "/Users/chuwenjie/CLionProjects/DUD/exp";
    } else {
        dictionary = "./exp/" + to_string(width) + '*' + to_string(height);
        out_dict = "./exp";
    }
    DIR *dir;
    struct dirent *ptr;
    vector<string> name_posts;
    vector<int> shape_nums;
    vector<string> filelist;
    vector<int> a_num_s;
    const char *p = dictionary.c_str();
    if ((dir = opendir(p)) == NULL) {
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL) {
        if (string(ptr->d_name).compare(0, 4, "grid") == 0)    //file
        {
            string temp;
            if (RUN_ENV == "WIN") {
                temp = dictionary + '\\' + ptr->d_name;
            } else {
                temp = dictionary + '/' + ptr->d_name;
            }
            filelist.push_back(temp);
            string _post = ptr->d_name;
            int shape_num = _post[5] - '0';
            shape_nums.push_back(shape_num);
            int a_num = atoi(_post.substr(7, _post.size() - 11).c_str());
            a_num_s.push_back(a_num);
            _post = _post.substr(4, _post.size() - 4);
            name_posts.push_back(_post);
        }
    }
    closedir(dir);

    center = Cloud(width,height,measure_scale);
    for (int f = 0; f < filelist.size(); f++) {
        vector<vector<int>> agent_init_pos_in_a_shape;
        agent_init_pos_in_shapes.push_back(agent_init_pos_in_a_shape);

        fstream infile;
        infile.open(filelist[f], ios::in);
        if (!infile) {
            cout << "open failed" << endl;
            exit(1);
        }
        int i = 0;
        int j = height - 1;
        int min_i = width;
        int min_j = height;
        while (!infile.eof() and j >= 0) {
            i = 0;
            while (!infile.eof() and i < width) {
                infile >> center.dt.grids[i][j];
                if (center.dt.grids[i][j] == 1) {
                    vector<int> pos = {i, j};
                    center.dt.target_poses.push_back(pos);
                    if (j <= min_j) {
                        if (j < min_j) {
                            min_j = j;
                            min_i = i;
                        } else {
                            if (i < min_i) {
                                min_i = i;
                            }
                        }
                    }
                }
                i++;
            }
            j--;
        }
        infile.close();

        int shape_agent_num;
        for (int k = 0; k < a_num_s[f]; k++) {
            swarm.push_back(Agent());
        }
        srand(time(NULL));
        int rand_more = 4;
        if (a_mode == more_a) {
            //如果是多,则为N+3~N+0.05N+3
            for (int i = 0; i < rand_more + 3; i++) {
                swarm.push_back(Agent());
            }
        } else if (a_mode == equal_a) {
            //如果是正好,由于seed robot中只有v3在目标内,所以至少为N+3
            for (int i = 0; i < 3; i++) {
                swarm.push_back(Agent());
            }
        } else {
            rand_more = rand_more - 3;
            //如果是少,则为N+3-0.05N~N+3
            if (rand_more > 0) {
                for (int i = 0; i < rand_more; i++) {
                    swarm.erase(swarm.end());
                }
            } else if (rand_more < 0) {
                for (int i = 0; i < -rand_more; i++) {
                    swarm.push_back(Agent());
                }
            }
        }
        shape_agent_num = swarm.size();
        center.set_a_num(shape_agent_num);

        vector<int> action_order(shape_agent_num, -1);
        for(int k=0;k<shape_agent_num;k++){
            action_order[k]=k;
        }
        //initialize seed robot v0-v3, noting that only v3 is within the target shape,
        //so the total number of agent for n-n shape is at least n+3 agent.
        agent_init_pos_in_shapes[f].push_back({min_i, min_j - 1});
        agent_init_pos_in_shapes[f].push_back({min_i + 1, min_j - 1});
        agent_init_pos_in_shapes[f].push_back({min_i, min_j - 2});
        agent_init_pos_in_shapes[f].push_back({min_i, min_j});
        //初始化剩余agent位置
        bool is_enough = initialize_no_seed_agent_positions(f, min_i, min_j, shape_agent_num);
        if (not is_enough) {
            cout << "Experiment failed!" << endl;
            continue;
        }

        int exp_num = 5;
        const int THREAD_NUM = TEMP_THREAD_NUM>shape_agent_num?shape_agent_num:TEMP_THREAD_NUM;

        double exp_avg_iter = 0;
        double exp_avg_iter_t = 0;
        int valid_exp = 0;
        int minor_valid_exp = 0;
        double minor_exp_avg_iter = 0;
        double minor_exp_avg_iter_t = 0;
        double exp_avg_similarity = 0;

        string out_arg;
        if (RUN_ENV == "WIN") {
            out_arg = out_dict + "\\args_" + to_string(width) + "_" +
                      to_string(height) + name_posts[f];
        } else {
            out_arg = out_dict + "/args_" + to_string(width) + "_" +
                      to_string(height) + name_posts[f];
        }
        ofstream outarg(out_arg, ios::app);



        for(int e=0; e<exp_num; e++) {
            //在每轮实验开始初始化所有agent的位置
            swarm[0].set_config(0,agent_init_pos_in_shapes[f][0][0], agent_init_pos_in_shapes[f][0][1], 2);
            swarm[1].set_config(1,agent_init_pos_in_shapes[f][1][0], agent_init_pos_in_shapes[f][1][1], 2);
            swarm[2].set_config(2, agent_init_pos_in_shapes[f][2][0], agent_init_pos_in_shapes[f][2][1], 2);
            swarm[3].set_config(3, agent_init_pos_in_shapes[f][3][0], agent_init_pos_in_shapes[f][3][1], 2);
            for (int s = 4; s < agent_init_pos_in_shapes[f].size(); s++) {
                swarm[s].set_config(s, agent_init_pos_in_shapes[f][s][0], agent_init_pos_in_shapes[f][s][1], 2);
            }

            string out_name;
            if (RUN_ENV == "WIN") {
                out_name = out_dict + "\\poses_" + to_string(e) + "_" +
                           to_string(width) + "_" + to_string(height) +
                           name_posts[f];
            } else {
                out_name = out_dict + "/poses_"  + to_string(e) + "_" +
                           to_string(width) + "_" + to_string(height) +
                           name_posts[f];
            }
            ofstream outfile(out_name, ios::app);

            //record the initialization
            outfile << "arguments: " << width << ' ' << height << ' ' << shape_agent_num << ' ' << THREAD_NUM
                    << endl;
            outfile << "agent positions:" << endl;
            for (int k = 0; k < shape_agent_num; k++) {
                if (k < shape_agent_num - 1) {
                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
                } else {
                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
                }
            }
            outfile << endl;

            vector<thread> threads;
            default_random_engine generator{random_device{}()};

            int terminal = 0;
            int minor_terminal = 0;
            int minor_minor_terminal = 0;
            clock_t startT, endT;
            vector<double> dec_times;
            center.calculate_dt_gradient();
            center.ac_tar = shape_agent_num-1;
            vector<int> ac_tar_decay;
            ac_tar_decay.push_back(center.ac_tar);

            cout<<"tar: "<<shape_agent_num<<" "<<center.dt.target_poses.size()<<endl;

            while (center.ac_tar > rand_more+3 && terminal < 1000 && minor_minor_terminal<500) {
                cout << center.ac_tar << endl;
                startT = clock();
                terminal += 1;

                //将agent的移动分配给不同的线程控制
                int left = int(shape_agent_num % THREAD_NUM);
                int alloc = 0;
                int s_ids[THREAD_NUM];
                int e_ids[THREAD_NUM];
                srand(time(NULL));
                for (int k = 0; k < THREAD_NUM; k++) {
                    s_ids[k] = -1, e_ids[k] = -1;
                    if (left > alloc) {
                        s_ids[k] = k * (int(shape_agent_num / THREAD_NUM) + 1);
                        e_ids[k] = s_ids[k] + int(shape_agent_num / THREAD_NUM);
                        alloc++;
                    } else {
                        s_ids[k] = alloc * (int(shape_agent_num / THREAD_NUM) + 1) +
                                   (k - alloc) * int(shape_agent_num / THREAD_NUM);
                        e_ids[k] = s_ids[k] + int(shape_agent_num / THREAD_NUM) - 1;
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
// comment 0617
                vector<int> line(height, -1);
                vector<vector<int>> array(width);
                for (int k = 0; k < array.size(); k++) {
                    array[k].assign(line.begin(), line.end());
                }
                center.agent_poses.swap(array);
//                unsigned seed = std::chrono::system_clock::now ().time_since_epoch ().count ();
//                std::shuffle (action_order.begin (), action_order.end (), std::default_random_engine (seed));
//                for (int k = 0; k < shape_agent_num; k++) {
//                    swarm[action_order[k]].take_action();
//                }

                endT = clock();
                dec_times.push_back((double) (endT - startT));

                center.ac_tar = 0;
                for (int k = 0; k < shape_agent_num; k++) {
                    if (center.dt.grids[swarm[k].pos_x][swarm[k].pos_y] == 0) {
                        center.ac_tar += 1;
                    }
                }
                ac_tar_decay.push_back(center.ac_tar);

                //record new positions for all agents
                outfile << "agent positions:" << endl;
                for (int k = 0; k < shape_agent_num; k++) {
                    if (k < shape_agent_num - 1) {
                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
                    } else {
                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
                    }
                }
                outfile << endl;

                if (center.ac_tar <= 0.3*shape_agent_num) {
                    minor_terminal += 1;
                }
                if(center.ac_tar<=0.1*shape_agent_num){
                    minor_minor_terminal += 1;
                }
            }
            outfile.flush();
            outfile.close();

            double avg_t = 0;
            for (int k = 0; k < dec_times.size(); k++) {
                avg_t += dec_times[k];
            }
            double avg_iteration = avg_t / (dec_times.size() * CLOCKS_PER_SEC);

            vector<vector<int>> formed_shape;
            for(int w=0;w<width;w++){
                formed_shape.push_back(vector<int>());
                for(int h=0;h<height;h++){
                    if(center.agent_poses[w][h]>=0){
                        formed_shape[w].push_back(1);
                    }else{
                        formed_shape[w].push_back(0);
                    }
                }
            }
            double mse_similarity = getSimilarity(center.dt.grids,formed_shape);
            exp_avg_similarity = exp_avg_similarity + mse_similarity;
            outarg << "Experiment " << e << ":" << endl;
            outarg << "The average decision time for each iteration is: " << avg_iteration << "s." << endl;
            outarg << "Main: program exiting after " << terminal << " steps, and " << minor_terminal
                   << " steps for the last "<<int(shape_agent_num*0.3)<<" positions, the similarity is " <<mse_similarity<< endl;
            outarg << "Decay line:";
            for (int k = 0; k < ac_tar_decay.size(); k++) {
                outarg << ' ' << ac_tar_decay[k];
            }
            ac_tar_decay.clear();
            outarg << endl;
            cout << "End an experiment! Clearing..." << endl;

            vector<int> line(height, -1);
            vector<vector<int>> array(width);
            for (int k = 0; k < array.size(); k++) {
                array[k].assign(line.begin(), line.end());
            }
            center.agent_poses.swap(array);
            center.dt.reset_dt(width, height);

            if (center.ac_tar <= 0.1*shape_agent_num) {
                valid_exp += 1;
                exp_avg_iter += (terminal-minor_minor_terminal);
                exp_avg_iter_t += avg_iteration;
            }
            if (center.ac_tar <= 0.3*shape_agent_num) {
                minor_valid_exp += 1;
                minor_exp_avg_iter = minor_exp_avg_iter + (terminal - minor_terminal);
                minor_exp_avg_iter_t += avg_iteration;
            }
        }
        exp_avg_iter = exp_avg_iter / valid_exp;
        exp_avg_iter_t = exp_avg_iter_t / valid_exp;
        exp_avg_similarity = exp_avg_similarity / exp_num;
        minor_exp_avg_iter = minor_exp_avg_iter / minor_valid_exp;
        minor_exp_avg_iter_t = minor_exp_avg_iter_t / minor_valid_exp;
        cout << f << ": exp_avg_iter=" << exp_avg_iter << ", exp_avg_iter_time=" << exp_avg_iter_t << endl;
        outarg << endl << endl;
        outarg << "After " << exp_num << " experiments, for shape " << f <<", avg_similarity="<<exp_avg_similarity
               <<", "<<valid_exp<<" experiments success, exp_avg_iter=" << exp_avg_iter
               << ", exp_avg_iter_time=" << exp_avg_iter_t << ", minor_exp_avg_iter=" << minor_exp_avg_iter
               << ", minor_exp_avg_iter_time=" << minor_exp_avg_iter_t << endl;
        outarg.flush();
        outarg.close();
        swarm.clear();
        vector<vector<int>> tmp_tars;
        center.dt.target_poses.swap(tmp_tars);
    }
    return 0;
}

/*random initialization main*/
//int main() {
//    string dictionary, out_dict;
//    if (RUN_ENV == "WIN") {
//        dictionary =
//                "D:\\projects\\CLionProjects\\InitSettingGenerator\\display\\" + to_string(width) + '_' + to_string(height);
//        out_dict = "D:\\projects\\CLionProjects\\DUD\\exp";
//    } else if (RUN_ENV == "MAC") {
//        dictionary =
//                "/Users/chuwenjie/CLionProjects/InitSettingGenerator/display/" + to_string(width) + '*' + to_string(height);
//        out_dict = "/Users/chuwenjie/CLionProjects/DUD/exp";
//    } else {
//        dictionary = "./exp/" + to_string(width) + '*' + to_string(height);
//        out_dict = "./exp";
//    }
//    DIR *dir;
//    struct dirent *ptr;
//    vector<string> name_posts;
//    vector<int> shape_nums;
//    vector<string> filelist;
//    vector<int> a_num_s;
//    const char *p = dictionary.c_str();
//    if ((dir = opendir(p)) == NULL) {
//        perror("Open dir error...");
//        exit(1);
//    }
//    while ((ptr = readdir(dir)) != NULL) {
//        if (string(ptr->d_name).compare(0, 4, "grid") == 0)    //file
//        {
//            string temp;
//            if (RUN_ENV == "WIN") {
//                temp = dictionary + '\\' + ptr->d_name;
//            } else {
//                temp = dictionary + '/' + ptr->d_name;
//            }
//            filelist.push_back(temp);
//            string _post = ptr->d_name;
//            int shape_num = _post[5] - '0';
//            shape_nums.push_back(shape_num);
//            int a_num = atoi(_post.substr(7, _post.size() - 11).c_str());
//            a_num_s.push_back(a_num);
//            _post = _post.substr(4, _post.size() - 4);
//            name_posts.push_back(_post);
//        }
//    }
//    closedir(dir);
//
//    for (int f = 0; f < filelist.size(); f++) {
//        int a_num = a_num_s[f];
//        int t_num = a_num_s[f];
//        const int THREAD_NUM = TEMP_THREAD_NUM>a_num?a_num:TEMP_THREAD_NUM;
//        center = Cloud(a_num, t_num, width, height,measure_scale);
//        for (int k = 0; k < a_num; k++) {
//            swarm.push_back(Agent());
//        }
//        int exp_num = 20;
//
//        string out_arg;
//        if (RUN_ENV == "WIN") {
//            out_arg = out_dict + "\\args_" + to_string(width) + "_" +
//                      to_string(height) + name_posts[f];
//        } else {
//            out_arg = out_dict + "/args_" + to_string(width) + "_" +
//                      to_string(height) + name_posts[f];
//        }
//        ofstream outarg(out_arg, ios::app);
//
//        double exp_avg_iter = 0;
//        double exp_avg_iter_t = 0;
//        int valid_exp = 0;
//        int minor_valid_exp = 0;
//        double minor_exp_avg_iter = 0;
//        double minor_exp_avg_iter_t = 0;
//        double exp_avg_similarity = 0;
//
//        for(int e=0; e<exp_num; e++) {
//            fstream infile;
//            infile.open(filelist[f], ios::in);
//            if (!infile) {
//                cout << "open failed" << endl;
//                exit(1);
//            }
//
//            int i = 0;
//            int j = height - 1;
//            vector<int> valid_l;
//            while (!infile.eof() and j >= 0) {
//                i = 0;
//                while (!infile.eof() and i < width) {
//                    infile >> center.dt.grids[i][j];
//                    if (center.dt.grids[i][j] == 1) {
//                        vector<int> pos = {i, j};
//                        center.dt.target_poses.push_back(pos);
//                    }else {
//                        valid_l.push_back(i * height + j);
//                    }
//                    i++;
//                }
//                j--;
//            }
//            infile.close();
//
//            int max_size = valid_l.size();
//            srand((unsigned) time(NULL));
//            for (int r = 0; r < a_num; r++) {
//                int rand_p = rand() % max_size;
//                int px = (int) valid_l[rand_p] / height;
//                int py = (int) valid_l[rand_p] % height;
//                swarm[r].set_config(r, px, py, 2);
//                center.agent_poses[px][py] = r;
//                int tmp = valid_l[max_size - 1];
//                valid_l[max_size - 1] = valid_l[rand_p];
//                valid_l[rand_p] = tmp;
//                max_size--;
//            }
//
//            string out_name;
//            if (RUN_ENV == "WIN") {
//                out_name = out_dict + "\\poses_" + to_string(e) + "_" +
//                           to_string(width) + "_" + to_string(height) +
//                           name_posts[f];
//            } else {
//                out_name = out_dict + "/poses_"  + to_string(e) + "_" +
//                           to_string(width) + "_" + to_string(height) +
//                           name_posts[f];
//            }
//            ofstream outfile(out_name, ios::app);
//            outfile << "arguments: " << width << ' ' << height << ' ' << a_num << ' ' << THREAD_NUM
//                    << endl;
//
//            //record initialization
//            outfile << "agent positions:" << endl;
//            for (int k = 0; k < a_num; k++) {
//                if (k < a_num - 1) {
//                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
//                } else {
//                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
//                }
//            }
//            outfile << endl;
//
//            vector<thread> threads;
//            default_random_engine generator{random_device{}()};
//
//            int terminal = 0;
//            int minor_terminal = 0;
//            int minor_minor_terminal = 0;
//            center.calculate_dt_gradient();
//            center.ac_tar = 0;
//            for (int k = 0; k < a_num; k++) {
//                if (center.dt.grids[swarm[k].pos_x][swarm[k].pos_y] == 0) {
//                    center.ac_tar += 1;
//                }
//            }
//            vector<int> ac_tar_decay;
//            ac_tar_decay.push_back(center.ac_tar);
//            clock_t startT, endT;
//            vector<double> dec_times;
//            while (center.ac_tar > 0 && terminal < 1000 && minor_minor_terminal<500) {
//                cout << center.ac_tar << endl;
//                startT = clock();
//                terminal += 1;
//
//                int left = int(a_num % THREAD_NUM);
//                int alloc = 0;
//                int s_ids[THREAD_NUM];
//                int e_ids[THREAD_NUM];
//                srand(time(NULL));
//                for (int k = 0; k < THREAD_NUM; k++) {
//                    s_ids[k] = -1, e_ids[k] = -1;
//                    if (left > alloc) {
//                        s_ids[k] = k * (int(a_num / THREAD_NUM) + 1);
//                        e_ids[k] = s_ids[k] + int(a_num / THREAD_NUM);
//                        alloc++;
//                    } else {
//                        s_ids[k] = alloc * (int(a_num / THREAD_NUM) + 1) + (k - alloc) * int(a_num / THREAD_NUM);
//                        e_ids[k] = s_ids[k] + int(a_num / THREAD_NUM) - 1;
//                    }
//                }
//                for (int k = 0; k < THREAD_NUM; k++) {
//                    threads.emplace_back(chooseActions, k, s_ids[k], e_ids[k]);
//                }
//                // 等待其他线程join
//                for (int k = 0; k < THREAD_NUM; k++) {
//                    threads[k].join();
//                }
//                threads.clear();
//
//                vector<int> line(height, -1);
//                vector<vector<int>> array(width);
//                for (int k = 0; k < array.size(); k++) {
//                    array[k].assign(line.begin(), line.end());
//                }
//                center.agent_poses.swap(array);
//
////                for (int k = 0; k < THREAD_NUM; k++) {
////                    threads.emplace_back(takeActions, k, s_ids[k], e_ids[k]);
////                }
////                // 等待其他线程join
////                for (int k = 0; k < THREAD_NUM; k++) {
////                    threads[k].join();
////                }
////                threads.clear();
//                for (int k = 0; k < a_num; k++) {
//                    swarm[k].take_action();
//                }
//                center.ac_tar = 0;
//                for (int k = 0; k < a_num; k++) {
//                    if (center.dt.grids[swarm[k].pos_x][swarm[k].pos_y] == 0) {
//                        center.ac_tar += 1;
//                    }
//                }
//                endT = clock();
//                dec_times.push_back((double) (endT - startT));
//
//                //record new positions for all agents
//                outfile << "agent positions:" << endl;
//                for (int k = 0; k < a_num; k++) {
//                    if (k < a_num - 1) {
//                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
//                    } else {
//                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
//                    }
//                }
//                outfile << endl;
//
//                ac_tar_decay.push_back(center.ac_tar);
//                if (center.ac_tar <= 0.3*a_num) {
//                    minor_terminal += 1;
//                }
//                if(center.ac_tar<=0.1*a_num){
//                    minor_minor_terminal += 1;
//                }
//            }
//            double avg_t = 0;
//            for (int k = 0; k < dec_times.size(); k++) {
//                avg_t += dec_times[k];
//            }
//            double avg_iteration = avg_t / (dec_times.size() * CLOCKS_PER_SEC);
//
//            outfile.flush();
//            outfile.close();
//
//            vector<vector<int>> formed_shape;
//            for(int w=0;w<width;w++){
//                formed_shape.push_back(vector<int>());
//                for(int h=0;h<height;h++){
//                    if(center.agent_poses[w][h]>=0){
//                        formed_shape[w].push_back(1);
//                    }else{
//                        formed_shape[w].push_back(0);
//                    }
//                }
//            }
//            double mse_similarity = getSimilarity(center.dt.grids,formed_shape);
//            exp_avg_similarity = exp_avg_similarity + mse_similarity;
//            outarg << "Experiment " << e << ":" << endl;
//            outarg << "The average decision time for each iteration is: " << avg_iteration << "s." << endl;
//            outarg << "Main: program exiting after " << terminal << " steps, and " << minor_terminal
//                   << " steps for the last "<<int(a_num*0.3)<<" positions, the similarity is " <<mse_similarity<< endl;
//            outarg << "Decay line:";
//            for (int k = 0; k < ac_tar_decay.size(); k++) {
//                outarg << ' ' << ac_tar_decay[k];
//            }
//            ac_tar_decay.clear();
//            outarg << endl;
//            cout << "End an experiment! Clearing..." << endl;
//
//            vector<int> line(height, -1);
//            vector<vector<int>> array(width);
//            for (int k = 0; k < array.size(); k++) {
//                array[k].assign(line.begin(), line.end());
//            }
//            center.agent_poses.swap(array);
//
//            if (center.ac_tar <= 0.1*a_num) {
//                valid_exp += 1;
//                exp_avg_iter += (terminal-minor_minor_terminal);
//                exp_avg_iter_t += avg_iteration;
//            }
//            if (center.ac_tar <= 0.3*a_num) {
//                minor_valid_exp += 1;
//                minor_exp_avg_iter = minor_exp_avg_iter + (terminal - minor_terminal);
//                minor_exp_avg_iter_t += avg_iteration;
//            }
//        }
//        exp_avg_iter = exp_avg_iter / valid_exp;
//        exp_avg_iter_t = exp_avg_iter_t / valid_exp;
//        exp_avg_similarity = exp_avg_similarity / exp_num;
//        minor_exp_avg_iter = minor_exp_avg_iter / minor_valid_exp;
//        minor_exp_avg_iter_t = minor_exp_avg_iter_t / minor_valid_exp;
//        cout << f << ": exp_avg_iter=" << exp_avg_iter << ", exp_avg_iter_time=" << exp_avg_iter_t << endl;
//        outarg << endl << endl;
//        outarg << "After " << exp_num << " experiments, for shape " << f <<", avg_similarity="<<exp_avg_similarity
//               <<", "<<valid_exp<<" experiments success, exp_avg_iter=" << exp_avg_iter
//               << ", exp_avg_iter_time=" << exp_avg_iter_t << ", minor_exp_avg_iter=" << minor_exp_avg_iter
//               << ", minor_exp_avg_iter_time=" << minor_exp_avg_iter_t << endl;
//        outarg.flush();
//        outarg.close();
//        swarm.clear();
//        center.dt.target_poses.clear();
//    }
//    return 0;
//}