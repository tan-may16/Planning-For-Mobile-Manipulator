#include <queue>
#include <unordered_map>
#include <iostream>
#include <bits/stdc++.h>
#include "state.h"
#include <vector>
#include <stack>
#include <chrono>
#include <fstream>
#include <cmath>
#include <filesystem>

using namespace std::chrono;
using namespace std;

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y)*XSIZE + (X))
// #define GETMAPINDEX_3D(X, Y, THETA, XSIZE, YSIZE, THETASIZE) (((Y)*XSIZE + (X))*THETASIZE + (int)((THETA)/45)) 
// #define GETMAPINDEX_3D(X, Y, THETA, XSIZE, YSIZE, THETASIZE) (((Y)*XSIZE + (X)) + (((THETA)/45))*(XSIZE )*(YSIZE )) 
// #define GETMAPINDEX_3D(X, Y, THETA, XSIZE, YSIZE, THETASIZE) (X + (Y * XSIZE) + (THETA * XSIZE * YSIZE)) 

#define NUMOFDIRS 8
#define NUMOFDIRS_3D 3

double const pi = 3.14159;

int GETMAPINDEX_3D(int X, int Y,int THETA,int XSIZE,int YSIZE,int THETASIZE)
{
    return (X + (Y * XSIZE) + (THETA * XSIZE * YSIZE/45)) ;
}

bool Is_valid(double* map, state* current, int dx, int dy, int x_size, int y_size, int collision_thresh = 100)
{
    if (current->x - dx < 0 || current->x + dx > x_size || current->y - dy < 0 || current->y + dy > y_size) return false;
    // return true;
    for (int j=dy;j>= (-dy);j--)
    {
        int succ1 = (int)GETMAPINDEX(current->x + dx, current->y + j, x_size,y_size);
        int succ2 = (int)GETMAPINDEX(current->x - dx, current->y - j, x_size,y_size);
        
        if (map[succ1]>= collision_thresh || map[succ2]>= collision_thresh || map[succ1] == 50 || map[succ2] == 50) return false;
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
    
    int x_size = map_size.first;
    int y_size = map_size.second;
    cout<< "Map Size: "<<x_size << " "<< y_size << endl;
    std::ifstream input(file_path);
    //check if file is open
    if (!input.is_open())
    {
        cout << "Error opening file";
        exit (1);
    }
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

void A_star_2D(int dX[],int dY[], int Gx, int Gy, double*map, int collision_thresh, int x_size, int y_size, int Rx, int Ry, unordered_map <int , state*> &OPEN_STATES, vector<state*> &path, bool policy=true, int weight = 1, bool backtrack = true, bool check_validity= true)
{
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

    while (!OPEN.empty())
    {
        
        current = OPEN.top();
        if (policy == false && current->x == Gx && current->y == Gy) 
        {
            cout<< "Goal Cost: " << current->g_cost<<endl;
            break;
        }
        
        OPEN.pop();
        current->state_expanded = true;

        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int ind = GETMAPINDEX(current->x + dX[dir],current->y + dY[dir],x_size,y_size);

            if (OPEN_STATES.count(ind) == 0) successor = new state(current->x + dX[dir],current->y + dY[dir], Rtheta);
            else successor = OPEN_STATES[ind];
            if (check_validity)
            {
                if (successor->x >= 0 && successor->x < x_size && successor->y >= 0 && successor->y < y_size && Is_valid(map, successor, 3, 3, x_size, y_size))
                {
                    int index_succ = GETMAPINDEX(successor->x,successor->y,x_size,y_size);
                    // int cost = (int)map[index_succ];
                    int cost;
                    if (abs(dX[dir]) + abs(dY[dir]) ==2) cost = 1.5;
                    else cost = 1;
                    if ((cost >= 0) && (map[index_succ] < collision_thresh) && successor->state_expanded == false  && (map[index_succ] != 50)) 
                    {
                        if (successor->g_cost > current->g_cost +cost)
                        {                 
                            successor->h_cost = successor->h_cost = weight*heurastic_2D_optimal(successor->x,successor->y,Gx,Gy);
                            successor->g_cost = current->g_cost +cost;
                            successor->f_cost = successor->g_cost + successor->h_cost;
                            successor->parent = current;

                            if (OPEN_STATES.count(ind) == 0) OPEN.push(successor);
                            OPEN_STATES[index_succ] = successor;
                            
                        }
                    }
                }
            }
            else
            {
                if (successor->x >= 0 && successor->x < x_size && successor->y >= 0 && successor->y < y_size)
                {
                    int index_succ = GETMAPINDEX(successor->x,successor->y,x_size,y_size);
                    // TODO: check for diagonal cost
                    int cost;
                    if (abs(dX[dir]) + abs(dY[dir]) ==2) cost = 1.5;
                    else cost = 1;
                    if ((cost >= 0) && (map[index_succ] < collision_thresh) && successor->state_expanded == false && (map[index_succ] != 50)) 
                    {
                        if (successor->g_cost > current->g_cost +cost)
                        {                 
                            successor->h_cost = successor->h_cost = weight*heurastic_2D_optimal(successor->x,successor->y,Gx,Gy);
                            successor->g_cost = current->g_cost +cost;
                            successor->f_cost = successor->g_cost + successor->h_cost;
                            successor->parent = current;

                            if (OPEN_STATES.count(ind) == 0) OPEN.push(successor);
                            OPEN_STATES[index_succ] = successor;
                            
                        }
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
            // cout<<"policy found"<<endl;
            auto end_time = high_resolution_clock::now();
            auto duration_time = duration_cast<milliseconds>(end_time - start_time);
            // cout<< ("Time required for policy: %d milliseconds",duration_time.count())<<endl;
            // cout<< ("No. of states expanded: %d ",OPEN_STATES.size())<<endl;
        }
    if (policy == false && backtrack == true) get_path(current, path);

}

// int sign(double x)
// {
//     if(x>=(1/sqrt(2))) return 1;
//     else if (x > -1/sqrt(2) && x< 1/sqrt(2)) return 0;
//     else return -1;
// }
// struct direction
// {
//     int dX[3];
//     int dY[3];
// };
pair <array<int,NUMOFDIRS_3D>,array<int,NUMOFDIRS_3D>> get_dx_dy(int theta)
{
    
    array<int,NUMOFDIRS_3D> dX;
    array<int,NUMOFDIRS_3D> dY;
    // dX = {sign(sin((theta +315)*pi/180)) ,  sign(sin((theta)*pi/180)), sign(sin((theta + 45)*pi/180))};
    // dY = {sign(cos((theta +315)*pi/180)),  sign(cos((theta)*pi/180)), sign(cos((theta + 45)*pi/180))};
    // return make_pair(dX,dY);
    if (theta == 0)
    {
        // direction d;
        // d.dX = 
        dX = {1,1,1};
        dY = {-1,0,1};
        return make_pair(dX,dY);
    }
    else if (theta == 315)
    {
        dX = {1,1,0};
        dY = {0,1,1};
        return make_pair(dX,dY);
    }
    else if (theta == 270)
    {
        dX = {1,0,-1};
        dY = {1,1,1};
        return make_pair(dX,dY);
    }
    else if (theta == 225)
    {
        dX = {0,-1,-1};
        dY = {1,1,0};
        return make_pair(dX,dY);
    }
    else if (theta == 180)
    {
        dX = {-1,-1,-1};
        dY = {1,0,1};
        return make_pair(dX,dY);
    }
    else if (theta == 135)
    {
        dX = {-1,-1,0};
        dY = {0,1,-1};
        return make_pair(dX,dY);
    }
    else if (theta == 90)
    {
        dX = {-1,0,1};
        dY = {1,-1,-1};
        return make_pair(dX,dY);
    }
    else if (theta == 45)
    {
        dX = {0,1,1};
        dY = {-1,-1,0};
        return make_pair(dX,dY);
    }
    else
    {
        cout<< "Problem with given angle"<<endl;
        dX = {0,0,0};
        dY = {0,0,0};
        return make_pair(dX,dY);
    }
    
}

void shift_coordinates(float &x, float &y, pair<int, int> map_size,const float& resolution)
{
    float scaled_x_size = map_size.first/2;
    float scaled_y_size = map_size.second/2;
    x = x/resolution + scaled_x_size;
    y = -y/resolution + scaled_y_size;
}

bool Is_valid_3D(state* current, double * map,int x_size, int y_size, int collision_thresh)
{
    array<int,NUMOFDIRS_3D> dX;
    array<int,NUMOFDIRS_3D> dY;
    array<int,NUMOFDIRS_3D+2> dXD;
    array<int,NUMOFDIRS_3D+2> dYD;
    int theta = current->theta;
    if (theta == 0)
    {
        // direction d;
        // d.dX = 
        dX = {0,1,0};
        dY = {-1,0,1};
        
    }
    else if (theta == 315)
    {
        dXD = {1,1,0,-1,1};
        dYD = {0,1,1,1,-1};
       
    }
    else if (theta == 270)
    {
        dX = {1,0,-1};
        dY = {0,1,0};
        
    }
    else if (theta == 225)
    {
        dXD = {0,-1,-1,-1,1};
        dYD = {1,1,0,-1,-1};
        
    }
    else if (theta == 180)
    {
        dX = {0,-1,0};
        dY = {1,0,-1};
        
    }
    else if (theta == 135)
    {
        dXD = {-1,-1,0,-1,1};
        dYD = {0,1,-1,1,-1};
        
    }
    else if (theta == 90)
    {
        dX = {-1,0,1};
        dY = {0,-1,0};
        
    }
    else if (theta == 45)
    {
        dXD = {0,1,1,-1,1};
        dYD = {-1,-1,0,-1,1};
        
    }
    if (theta % 90 == 0)
    {
        for (int i=0;i<3;i++)
        {
            int ind = GETMAPINDEX(current->x + 3 * dX[i],current->y + 3 * dY[i],x_size,y_size);
            if (map[ind] >= collision_thresh || map[ind] == 50) return false;
        }
        return true;
    }
    else
    {
        for (int i=0;i<5;i++)
        {
            int ind = GETMAPINDEX(current->x + 3 * dXD[i],current->y + 3 * dYD[i],x_size,y_size);
            if (map[ind] >= collision_thresh || map[ind] == 50) return false;
        }
        return true;
    }
    return true;
}


void A_star_3D(int dTheta_3D[], int Gx, int Gy, int Gtheta, int Rx, int Ry, int Rtheta, double*map, int collision_thresh, int x_size, int y_size, int theta_size,  unordered_map <int , state*> &OPEN_STATES_3D, vector<state*> &path_3D, bool policy=true, int weight = 1)
{
    auto start_time = high_resolution_clock::now();
    int while_count = 0;
    priority_queue <state*,vector<state*>,decltype(cmp)> OPEN(cmp);
    
    state *goal = new state(Gx,Gy, Gtheta);
    goal->g_cost = 0;

    state * start = new state(Rx,Ry,Rtheta);

    unordered_map <int , state*> OPEN_STATES_3D_temp;
    vector<state*> path_3D_temp;

    int dX_2D[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY_2D[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    A_star_2D(dX_2D, dY_2D, start->x, start->y, map, 257, x_size, y_size, goal->x, goal->y, OPEN_STATES_3D_temp, path_3D_temp, true, weight = 1, false, false);
    // cout<< OPEN_STATES_3D_temp.size()<<endl;
    state *successor = new state();
    state *current = new state(Rx, Ry, Rtheta);
    current->g_cost = 0;
    OPEN.push(current);
    int index = GETMAPINDEX_3D(current->x,current->y,current->theta,x_size,y_size,theta_size);
    OPEN_STATES_3D.insert(make_pair(index,current));
    // cout<< current->x << " " <<current->y << " " << current->theta<<endl;
    // cout<< goal->x << " " <<goal->y << " "<< goal->theta<<endl;
    // cout<<"Is goal " <<(current->x == Gx && current->y == Gy && current->theta == Gtheta) <<endl;
    while (!OPEN.empty())
    {
        // cout<< "Open list size: "<<OPEN.size()<<endl;
        // if (while_count%100000 == 0) cout<<while_count<<endl;
        current = OPEN.top();
        if (policy == false && current->x == Gx && current->y == Gy && current->theta == Gtheta) 
        {
            cout<< "Cost of goal: "<<current->g_cost<<endl;
            break;
        }
                
        OPEN.pop();
        current->state_expanded = true;
        // cout<< current->x << " "<<current->y <<" "<<current->theta<<endl;
        pair <array<int,NUMOFDIRS_3D>,array<int,NUMOFDIRS_3D>> delta = get_dx_dy(current->theta);
        array<int,NUMOFDIRS_3D> dX = delta.first;
        array<int,NUMOFDIRS_3D> dY = delta.second;
        for(int dir = 0; dir < NUMOFDIRS_3D; dir++)
        {
            // cout<<dir<<endl;
            // cout<< dTheta[dir]<<endl;
            int ind = GETMAPINDEX_3D(current->x + dX[dir],current->y + dY[dir],(current->theta + dTheta_3D[dir])%360,x_size,y_size,theta_size);
            // cout<<"Entered For loop"<<endl;
            // cout<< ind<<endl;
            // cout<<dX[dir]<<" "<<dY[dir] << "" <<endl;
            if (OPEN_STATES_3D.count(ind) == 0) successor = new state(current->x + dX[dir],current->y + dY[dir], current->theta + dTheta_3D[dir]);
            else 
            {
                successor = OPEN_STATES_3D[ind];
                // cout<<"old succ"<<endl;
            }
            // cout<< successor->x << " "<<successor->y <<" "<<successor->theta<<endl;
            
            if (successor->x >= 0 && successor->x < x_size && successor->y >= 0 && successor->y < y_size && successor->theta >=0 && successor->theta < 360)
            {
                int index_succ = GETMAPINDEX_3D(successor->x,successor->y,successor->theta,x_size,y_size,theta_size);
                int index_succ_2D = GETMAPINDEX(successor->x,successor->y,x_size,y_size);
                
                int cost;
                if (abs(dX[dir]) + abs(dY[dir]) == 2) cost = 1.5;
                else cost = 1;
                
                // cout<<"cost of successor"<<endl;
                if ((cost >= 0) && (map[index_succ_2D] < collision_thresh) && (map[index_succ_2D] != 50) && successor->state_expanded == false && Is_valid_3D(successor,map,x_size,y_size,collision_thresh)) 
                {
                    if (successor->g_cost > current->g_cost + cost)
                    {                 
                        // cout<<"Valid Successor"<<endl;
                        
                        // successor->h_cost = weight*OPEN_STATES_3D_temp[GETMAPINDEX(successor->x,successor->y,x_size,y_size)]->g_cost;
                        successor->g_cost = current->g_cost +cost;
                        successor->f_cost = successor->g_cost + successor->h_cost;
                        successor->parent = current;

                        if (OPEN_STATES_3D.count(index_succ) == 0) OPEN.push(successor);
                        OPEN_STATES_3D[index_succ] = successor;
                        // if (successor->y ==1 || successor->y == 0) cout<<successor->x<<" "<<successor->y<<" "<<successor->theta<< " "<<index_succ<<endl;
                        // cout<< "Added Successor"<<endl;
                        
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
    cout<< "Number of states in Open Queue: "<<OPEN.size()<<endl;
    cout<<"No. of States expanded: "<<OPEN_STATES_3D.size()<<endl;
    if (policy == false) 
    {
        get_path(current, path_3D);
        
    }


}




int main(int argc, char **argv)
{
    // take robot intial and goal position as input
    float Rx = argc > 1 ? atof(argv[1]) : 0;
    float Ry = argc > 2 ? atof(argv[2]) : 0;
    int Rtheta = argc > 3 ? atoi(argv[3]) : 0;
    float Gx = argc > 4 ? atof(argv[4]) : 0;
    float Gy = argc > 5 ? atof(argv[5]) : 0;
    int Gtheta = argc > 6 ? atoi(argv[6]) : 0;
    float resolution = argc > 7 ? atof(argv[7]) : 0.1;

    resolution = 1/resolution;

    string cwd = argv[0];
    cwd = cwd.substr(0, cwd.find_last_of("/\\"));
    cout << "\nCurrent working directory: " << cwd << endl;

    string map_file_path = cwd + "/office.txt";
    string map_save_path = cwd + "/office_map.txt";
    string path_2D_file_path = cwd + "/office_2d.txt";
    string path_3D_file_path = cwd + "/office_3d.txt";
    // pair <array<int,NUMOFDIRS_3D>,array<int,NUMOFDIRS_3D>> delta = get_dx_dy(0);
    // for (int i = 0;i<NUMOFDIRS_3D;i++)
    // {
    //     cout<< delta.first[i] << "," <<delta.second[i] <<endl;
    // }

    pair<int, int> map_size = get_dimensions(map_file_path);
    
    // Shifting ccordinates to pixel frame
    shift_coordinates(Rx, Ry, map_size, resolution);
    shift_coordinates(Gx, Gy, map_size, resolution);

    int Rx_pixel = (int)Rx;
    int Ry_pixel = (int)Ry;

    int Gx_pixel = (int)Gx;
    int Gy_pixel = (int)Gy;

    // print shifted coordinates
    cout << "Robot Initial Position: " << Rx_pixel << "," << Ry_pixel << "," << Rtheta << endl;
    cout << "Goal Position: " << Gx_pixel << "," << Gy_pixel << "," << Gtheta << endl;

    double* map = create_map(map_file_path, map_size);
    // Create and save own map format
    ofstream myfile_ (map_save_path);
    if (myfile_.is_open())
    {
        myfile_ << "#" << map_size.first << "," << map_size.second <<endl;
        for (int i =0;i< map_size.first;i++)
        {
            for (int j=0;j<map_size.second;j++)
            {
                myfile_ << i <<","<< j<<","<< map[(int)GETMAPINDEX(i,j,map_size.first,map_size.second)] <<endl;
            }
            
        }
        myfile_.close();
    }
    else cout << "Unable to open file";

    unordered_map <int , state*> OPEN_STATES;
    vector<state*> path;

    // int Gx,Gy,Gtheta,Rx,Ry,Rtheta;
    int collision_thresh,weight;
    bool policy,get_path, check_validity;
    collision_thresh = 100;
    weight = 1;
    policy = false;
    get_path = true;
    check_validity = true;
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    

     // 2D A*
     // Start clock
    auto start = high_resolution_clock::now();
    cout << "2D A* Implemented:"<<endl;
    A_star_2D(dX, dY, Gx_pixel, 
            Gy_pixel, map, collision_thresh, 
            map_size.first, map_size.second, 
            Rx_pixel, Ry_pixel, OPEN_STATES, 
            path, policy, weight, get_path, 
            check_validity);
    // Stop clock   
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "Time taken by 2D A Star: "
         << duration.count() << " microseconds" << endl;

    ofstream myfile_2d (path_2D_file_path);
    if (myfile_2d.is_open())
    {
        for (int i =0;i<path.size();i++)
        {
            myfile_2d << path[i]->x <<","<< path[i]->y<<endl;
        }
        myfile_2d.close();
    }
    else cout << "Unable to open file";
    
    // 3D A*
    int dTheta_3D[NUMOFDIRS_3D] = {45, 0, 315};
    unordered_map <int , state*> OPEN_STATES_3D;
    vector<state*> path_3D;

    // Start clock
    start = high_resolution_clock::now();
    cout << endl <<"3D A* Implemented:"<<endl;
    A_star_3D(dTheta_3D, Gx_pixel, Gy_pixel, 
            Gtheta, Rx_pixel, Ry_pixel, 
            Rtheta, map, collision_thresh, 
            map_size.first, map_size.second, 
            8,  OPEN_STATES_3D, path_3D, 
            false, weight);
    // Stop clock
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    cout << "Time taken by 3D A Star: "
         << duration.count() << " microseconds" << endl;
    cout<<"Path size: "<<path_3D.size()<<endl;

    ofstream myfile_3d (path_3D_file_path);
    if (myfile_3d.is_open())
    {
        for (int i =0;i<path_3D.size();i++)
        {
            myfile_3d << path_3D[i]->x <<","<< path_3D[i]->y << "," << path_3D[i]->theta<<endl;
        }
        myfile_3d.close();
    }
    else cout << "Unable to open file";

   
    return 0;
}


