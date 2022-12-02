#include <queue>
#include <unordered_map>
#include <iostream>
#include <bits/stdc++.h>
#include "state.h"
#include <vector>
#include <stack>
#include <chrono>
#include <fstream>
using namespace std::chrono;
using namespace std;

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y)*XSIZE + (X))
#define GETMAPINDEX_3D(X, Y, THETA, XSIZE, YSIZE, THETASIZE) (((Y)*XSIZE + (X))*THETASIZE + (THETA)) 
#define NUMOFDIRS 8
#define NUMOFDIRS_3D 8


bool Is_valid(double* map, state* current, int dx, int dy, int x_size, int y_size, int collision_thresh)
{
    if (current->x - dx < 0 || current->x + dx > x_size || current->y - dy < 0 || current->y + dy > y_size) return false;
    for (int i = dx;i>=0;i--)
    {
        for (int j=dy;j>=0;j--)
        {
            int succ1 = (int)GETMAPINDEX(current->x + i, current->y + j, x_size,y_size);
            int succ2 = (int)GETMAPINDEX(current->x - i, current->y - j, x_size,y_size);
            
            // state* successor_1 = new state(current->x + i, current->y + j, current->theta);
            // state* successor_2 = new state(current->x - i, current->y - j, current->theta);
            if (map[succ1]>= collision_thresh ||map[succ2]>= collision_thresh) return false;
        }
    }
    return true;
}
pair <int, int> get_dimensions(string file_path)
{
    int x_size,y_size;
    std::ifstream input(file_path);
    for( std::string line; getline( input, line );)
    {
        if (line.substr(0,1) == "#")
        {
            stringstream s(line);
            string x;
            string y;
            getline(s, x, ':');
            getline(s, x, ',');
            x_size = stoi(x);
            getline(s, y, ',');
            y_size = stoi(y);
            break;
        }
    }
    pair <int, int> map_size = make_pair(x_size,y_size);
    return map_size;

}
double* create_map(string file_path, pair <int, int> map_size)
{
    
    // stringstream s_stream(file_path);
    // int x_size,y_size;
    int x_size = map_size.first;
    int y_size = map_size.second;
    cout<< x_size << " "<< y_size << endl;
    std::ifstream input(file_path);
    // for( std::string line; getline( input, line );)
    // {
    //     if (line.substr(0,1) == "#")
    //     {
    //         stringstream s(line);
    //         string x;
    //         string y;
    //         getline(s, x, ':');
    //         getline(s, x, ',');
    //         x_size = stoi(x.substr(3));
    //         getline(s, y, ',');
    //         y_size = stoi(y);
    //         break;
    //     }
    // }
    double* map = new double[x_size*y_size];
    for( std::string line; getline( input, line );)
    {
        if (line.substr(0,1) != "#")
       
        {
            stringstream s(line);
            string substr_x;
            string substr_y;
            string substr_val;
            getline(s, substr_x, ',');
            getline(s, substr_y, ':');
            getline(s, substr_val, '(');
            getline(s, substr_val, ')');
            int x = stoi(substr_x);
            int y = stoi(substr_y);
            int val = stoi(substr_val);
            // cout<<val<<endl;
            map[(int)GETMAPINDEX(x,y,x_size,y_size)] = 255 - val;

        }
    }
    return map;
}

double heurastic_2D_optimal (int Rx, int Ry, int Gx, int Gy)
{
    int delta_x = Rx - Gx;
    int delta_y = Ry - Gy;

    double h = sqrt(2)*MIN(abs(delta_x),abs(delta_y)) + ((MAX(abs(delta_x),abs(delta_y)))-(MIN(abs(delta_x),abs(delta_y))));
    return h;
}

double heurastic_2D_diagonal (int Rx, int Ry, int Gx, int Gy)
{
    double h = MAX(abs(Rx-Gx),abs(Ry-Gy));
    return h;
}

auto cmp= [](const state* a, const state* b)
    {
        if (a->f_cost == b->f_cost) return a->h_cost < b->h_cost;
        return a->f_cost > b->f_cost;
    };

auto cmp_pair= [](const pair<int,int> a, const pair<int,int> b)
    {
        return a.second > b.second;
    };


void get_path(state* start, vector<state*> &path)
{
    state* temp = start;
    while(temp!=NULL)
    {
        path.push_back(temp);
        temp = temp->parent;
    }
}


stack <pair<int,int>> Trajectory;
priority_queue <pair<int,int>> target_g_cost;

void A_star_2D(int dX[],int dY[], int Gx, int Gy, double*map, int collision_thresh, int x_size, int y_size, int Rx, int Ry, unordered_map <int , state*> &OPEN_STATES, vector<state*> &path, bool policy=true, int weight = 1, bool backtrack = true)
{
    cout<<"Entered A*"<<endl;
    
    auto start_time = high_resolution_clock::now();
    int while_count = 0;
    priority_queue <state*,vector<state*>,decltype(cmp)> OPEN(cmp);
    // unordered_map <int , state*> OPEN_STATES;
    int Gtheta = 0;
    int Rtheta = 0;
    state *goal = new state(Gx,Gy, Gtheta);
    state *successor = new state();


    state *current = new state(Rx, Ry, Rtheta);
    current->g_cost = 0;
    OPEN.push(current);
    int index = GETMAPINDEX(current->x,current->y,x_size,y_size);
    OPEN_STATES.insert(make_pair(index,current));

    cout<<"Starting while loop"<<endl;
    while (!OPEN.empty())
    {
        
        current = OPEN.top();
        if (policy == false && current->x == Gx && current->y == Gy) break;
        
        OPEN.pop();
        current->state_expanded = true;

        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int ind = GETMAPINDEX(current->x + dX[dir],current->y + dY[dir],x_size,y_size);

            if (OPEN_STATES.count(ind) == 0) successor = new state(current->x + dX[dir],current->y + dY[dir], Rtheta);
            else successor = OPEN_STATES[ind];
            
            if (successor->x >= 0 && successor->x < x_size && successor->y >= 0 && successor->y < y_size)
            {
                int index_succ = GETMAPINDEX(successor->x,successor->y,x_size,y_size);
                int cost = (int)map[index_succ];
                if ((cost >= 0) && (cost < collision_thresh) && successor->state_expanded == false) 
                {
                    if (successor->g_cost > current->g_cost +(int)map[index_succ])
                    {                 
                        successor->h_cost = successor->h_cost = weight*heurastic_2D_diagonal(successor->x,successor->y,Gx,Gy);
                        successor->g_cost = current->g_cost +(int)map[index_succ];
                        successor->f_cost = successor->g_cost + successor->h_cost;
                        successor->parent = current;

                        if (OPEN_STATES.count(ind) == 0) OPEN.push(successor);
                        OPEN_STATES[index_succ] = successor;
                        
                    }
                }
            }
        }
        while_count++;

        if (while_count>10000000)
        {
            cout<<"While loop Cut"<<endl;
            cout<<"Too many states to handle"<<endl;
            break;
        }

    }
    if (while_count<10000000)
        {
            cout<<"policy found"<<endl;
            auto end_time = high_resolution_clock::now();
            auto duration_time = duration_cast<milliseconds>(end_time - start_time);
            cout<< ("Time required for policy: %d milliseconds",duration_time.count())<<endl;
            cout<< ("No. of states expanded: %d ",OPEN_STATES.size())<<endl;
        }
    if (policy == false && backtrack == true) get_path(current, path);

}









void A_star_3D(int dX[],int dY[], int dTheta[], int Gx, int Gy, int Gtheta, double*map, int collision_thresh, int x_size, int y_size, int theta_size, int Rx, int Ry, int Rtheta, unordered_map <int , state*> &OPEN_STATES_3D, vector<state*> &path_3D, bool policy=true, int weight = 1)
{
    auto start_time = high_resolution_clock::now();
    int while_count = 0;
    priority_queue <state*,vector<state*>,decltype(cmp)> OPEN(cmp);
    // unordered_map <int , state*> OPEN_STATES;
    // int Gtheta = 0;
    // int Rtheta = 0;

    state *goal = new state(Gx,Gy, Gtheta);
    state *successor = new state();


    state *current = new state(Rx, Ry, Rtheta);
    current->g_cost = 0;
    OPEN.push(current);
    int index = GETMAPINDEX_3D(current->x,current->y,current->theta,x_size,y_size,theta_size);
    OPEN_STATES_3D.insert(make_pair(index,current));


    while (!OPEN.empty())
    {
        if (policy == false && current->x == Gx && current->x == Gy && current->theta == Gtheta) break;
        
        current = OPEN.top();
        if (policy == false && current->x == Gx && current->y == Gy) break;
        
        OPEN.pop();
        current->state_expanded = true;

        for(int dir = 0; dir < NUMOFDIRS_3D; dir++)
        {
            int ind = GETMAPINDEX_3D(current->x + dX[dir],current->y + dY[dir],current->theta + dTheta[dir],x_size,y_size,theta_size);

            if (OPEN_STATES_3D.count(ind) == 0) successor = new state(current->x + dX[dir],current->y + dY[dir], current->theta);
            else successor = OPEN_STATES_3D[ind];
            // ###### theta limit?? ######## Getmapindex??
            if (successor->x >= 0 && successor->x < x_size && successor->y >= 0 && successor->y < y_size && successor->theta < theta_size && successor->theta >= 0)
            {
                int index_succ = GETMAPINDEX_3D(successor->x,successor->y,successor->theta,x_size,y_size,theta_size);
                int cost = (int)map[index_succ];
                if ((cost >= 0) && (cost < collision_thresh) && successor->state_expanded == false) 
                {
                    if (successor->g_cost > current->g_cost +(int)map[index_succ])
                    {                 
                        // successor->h_cost = successor->h_cost = weight*heurastic_2D_diagonal(successor->x,successor->y,Gx,Gy);
                        unordered_map <int , state*> OPEN_STATES_3D_temp;
                        vector<state*> path_3D_temp;
                        A_star_2D(dX, dY, goal->x, goal->y, map, collision_thresh, x_size, y_size, successor->x, successor->y, OPEN_STATES_3D_temp, path_3D_temp, policy=false, weight = 1, false);
                        successor->h_cost = OPEN_STATES_3D_temp[GETMAPINDEX(successor->x,successor->y,x_size,y_size)]->g_cost;
                        successor->g_cost = current->g_cost +(int)map[index_succ];
                        successor->f_cost = successor->g_cost + successor->h_cost;
                        successor->parent = current;
                        if (OPEN_STATES_3D.count(ind) == 0) OPEN.push(successor);
                        OPEN_STATES_3D[index_succ] = successor;
                        
                    }
                }
            }
        }
        while_count++;

        if (while_count>10000000)
        {
            cout<<"While loop Cut"<<endl;
            cout<<"Too many states to handle"<<endl;
            break;
        }

    }
    if (policy == false) get_path(current, path_3D);

}














// stack <pair<int,int>> A_star_3D(int dX[],int dY[], int dTheta[], int Gx, int Gy, int Gtheta, double*map, int collision_thresh, int x_size, int y_size, int theta_size, int Rx, int Ry, int Rtheta, bool policy=true, int weight = 1)
// {
//     auto start_time = high_resolution_clock::now();
//     int while_count = 0;
//     priority_queue <state*,vector<state*>,decltype(cmp)> OPEN(cmp);
//     unordered_map <int , state*> OPEN_STATES;
//     int Gtheta = 0;
//     int Rtheta = 0;

//     state *goal = new state(Gx,Gy, Gtheta);
//     state *successor = new state();


//     state *current = new state(Rx, Ry, Rtheta);
//     current->g_cost = 0;
//     OPEN.push(current);
//     int index = GETMAPINDEX(current->x,current->y,x_size,y_size);
//     OPEN_STATES.insert(make_pair(index,current));


//     while (!OPEN.empty())
//     {
//         current = OPEN.top();
//         if (policy == false && current->x == Gx && current->y == Gy) break;
        
//         OPEN.pop();
//         current->state_expanded = true;

//         for(int dir = 0; dir < NUMOFDIRS; dir++)
//         {
//             int ind = GETMAPINDEX(current->x + dX[dir],current->y + dY[dir],x_size,y_size);

//             if (OPEN_STATES.count(ind) == 0) successor = new state(current->x + dX[dir],current->y + dY[dir], Rtheta);
//             else successor = OPEN_STATES[ind];
            
//             if (successor->x >= 0 && successor->x <= x_size && successor->y >= 0 && successor->y <= y_size)
//             {
//                 int index_succ = GETMAPINDEX(successor->x,successor->y,x_size,y_size);
//                 int cost = (int)map[index_succ];
//                 if ((cost >= 0) && (cost < collision_thresh) && successor->state_expanded == false) 
//                 {
//                     if (successor->g_cost > current->g_cost +(int)map[index_succ])
//                     {                 
//                         successor->h_cost = successor->h_cost = weight*heurastic_2D_diagonal(successor->x,successor->y,Gx,Gy);
//                         successor->g_cost = current->g_cost +(int)map[index_succ];
//                         successor->f_cost = successor->g_cost + successor->h_cost;
//                         successor->parent = current;

//                         if (OPEN_STATES.count(ind) == 0) OPEN.push(successor);
//                         OPEN_STATES[index_succ] = successor;
                        
//                     }
//                 }
//             }
//         }
//         while_count++;

//         if (while_count>10000000)
//         {
//             cout<<"While loop Cut"<<endl;
//             cout<<"Too many states to handle"<<endl;
//             break;
//         }

//     }


//     // int final_index = (int)GETMAPINDEX(Rx,Ry,x_size,y_size);
//     // state * final_state = OPEN_STATES[final_index];
//     // if (policy == true)
//     // {

    
//     //     priority_queue <pair<int,int>,vector<pair<int,int>>,decltype(cmp_pair)> traj_cost(cmp_pair);
//     //     for (int i = 0; i<target_steps - 1; i++)
//     //     {
//     //         current = OPEN_STATES[GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)];
//     //         pair <int,int> cost_of_path = calculate_path_steps(i, map, current,Trajectory,x_size, y_size, target_steps, target_traj);
//     //         int time_diff = i - cost_of_path.second;
//     //         auto stop_time = high_resolution_clock::now();
//     //         auto duration_time = duration_cast<seconds>(stop_time - start_time);
//     //         if (time_diff > 0 + duration_time.count() )
//     //         {
//     //             if (weight == 1)
//     //             {
//     //                 int cell_cost = map[(int)GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)];
//     //                 int path_cost = OPEN_STATES[(int)GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)]->g_cost;
//     //                 int total_cost = path_cost + time_diff*cell_cost;
//     //                 cost_of_path.second = total_cost;
//     //                 traj_cost.push(cost_of_path);
//     //             }
//     //             else
//     //             {
//     //                 pair <int,int> actual_cost = calculate_path_cost(i, map, current,Trajectory,x_size, y_size, target_steps, target_traj);
//     //                 int cell_cost = map[(int)GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)];
                    
//     //                 int total_cost = actual_cost.second + time_diff*cell_cost;
//     //                 cost_of_path.second = total_cost;
//     //                 traj_cost.push(cost_of_path);
//     //             }
//     //         }
            
//     //     }
//     //     if (traj_cost.empty())
//     //     {
//     //         return Trajectory;
//     //     }
//     //     while(!Trajectory.empty())
//     //     {
//     //         Trajectory.pop();
//     //     }
//         // cout<<"Intersection at target_traj index ="<<traj_cost.top().first<<" ,"<<"Cost Estimate:"<<traj_cost.top().second<<endl;
//         // final_index = traj_cost.top().first;
//         // final_state = OPEN_STATES[(int)GETMAPINDEX(target_traj[final_index-1],target_traj[final_index-1 + target_steps],x_size,y_size)];
    
//     // }
//     // else
//     // {
//     //     while(!Trajectory.empty())
//     //     {
//     //         Trajectory.pop();
//     //     }
//     //     if (while_count>10000000)
//     //     {
//     //         return Trajectory;
//     //     }
//     //     else
//     //     {
//     //         final_index = (int)GETMAPINDEX(current->x ,current->y,x_size,y_size);
//     //         final_state = current;
//     //     }

//     // }
//     // pair <int,int> cost_of_path = calculate_path_steps(final_index, map, final_state,Trajectory,x_size, y_size, target_steps, target_traj,true);
//     // return Trajectory;


// }

int main()
{
    pair<int, int> map_size = get_dimensions("example.txt");
    cout<<"Got dimensions"<<endl;
    double* map = create_map("example.txt",map_size);
    cout<<"Returning map"<<endl;
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    cout<<"Created map"<<endl;
    unordered_map <int , state*> OPEN_STATES;
    vector<state*> path;
    
    A_star_2D(dX, dY, 300, 300, map, 256, map_size.first, map_size.second, 1, 1, OPEN_STATES, path, true, 1, true);
    cout<<path.size()<<endl;
    return 0;
}