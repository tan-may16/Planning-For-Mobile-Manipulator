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
        
        if (map[succ1]>= collision_thresh ||map[succ2]>= collision_thresh) return false;
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
    cout<< x_size << " "<< y_size << endl;
    std::ifstream input(file_path);
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
        if (policy == false && current->x == Gx && current->y == Gy) break;
        
        OPEN.pop();
        current->state_expanded = true;

        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int ind = GETMAPINDEX(current->x + dX[dir],current->y + dY[dir],x_size,y_size);

            if (OPEN_STATES.count(ind) == 0) successor = new state(current->x + dX[dir],current->y + dY[dir], Rtheta);
            else successor = OPEN_STATES[ind];
            if (check_validity)
            {
                if (successor->x >= 0 && successor->x < x_size && successor->y >= 0 && successor->y < y_size && Is_valid(map, successor, 1, 1, x_size, y_size))
                {
                    int index_succ = GETMAPINDEX(successor->x,successor->y,x_size,y_size);
                    int cost = (int)map[index_succ];
                    if ((cost >= 0) && (cost < collision_thresh) && successor->state_expanded == false) 
                    {
                        if (successor->g_cost > current->g_cost +(int)map[index_succ])
                        {                 
                            successor->h_cost = successor->h_cost = weight*heurastic_2D_optimal(successor->x,successor->y,Gx,Gy);
                            successor->g_cost = current->g_cost +(int)map[index_succ];
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
                    if (abs(dX[dir]) + abs(dY[dir]) ==2) cost = 2;
                    else cost = 1;
                    if ((cost >= 0) && (map[index_succ] < collision_thresh) && successor->state_expanded == false) 
                    {
                        if (successor->g_cost > current->g_cost +cost)
                        {                 
                            successor->h_cost = successor->h_cost = weight*heurastic_2D_diagonal(successor->x,successor->y,Gx,Gy);
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
            cout<< ("Time required for policy: %d milliseconds",duration_time.count())<<endl;
            cout<< ("No. of states expanded: %d ",OPEN_STATES.size())<<endl;
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
        cout<< "Error with angle"<<endl;
        dX = {0,0,0};
        dY = {0,0,0};
        return make_pair(dX,dY);
    }
    
}

void shift_coordinates(vector<state*> &path, int x_size, int y_size)
{
    for (int i=0; i<path.size();i++ )
    {
        path[i]->x -= x_size/2.0;
        path[i]->y -= y_size/2.0;
        path[i]->y = -path[i]->y;
        path[i]->theta *= 3.14159265/180.0;      
    }
}   


bool Is_valid_3D(state* current, double * map,int x_size, int y_size, int collision_thresh)
{
    array<int,NUMOFDIRS_3D> dX;
    array<int,NUMOFDIRS_3D> dY;
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
        dX = {1,1,0};
        dY = {0,1,1};
       
    }
    else if (theta == 270)
    {
        dX = {1,0,-1};
        dY = {0,1,0};
        
    }
    else if (theta == 225)
    {
        dX = {0,-1,-1};
        dY = {1,1,0};
        
    }
    else if (theta == 180)
    {
        dX = {0,-1,0};
        dY = {1,0,-1};
        
    }
    else if (theta == 135)
    {
        dX = {-1,-1,0};
        dY = {0,1,-1};
        
    }
    else if (theta == 90)
    {
        dX = {-1,0,1};
        dY = {0,-1,0};
        
    }
    else if (theta == 45)
    {
        dX = {0,1,1};
        dY = {-1,-1,0};
        
    }
    for (int i=0;i<3;i++)
    {
        int ind = GETMAPINDEX(current->x + dX[i],current->y+dY[i],x_size,y_size);
        if (map[ind] >= collision_thresh) return false;
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
    cout<< OPEN_STATES_3D_temp.size()<<endl;
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
        if (while_count%100000 == 0) cout<<while_count<<endl;
        current = OPEN.top();
        if (policy == false && current->x == Gx && current->y == Gy && current->theta == Gtheta) 
        {
            cout<< "Cost of goal"<<current->g_cost<<endl;
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
                if ((cost >= 0) && (map[index_succ_2D] < collision_thresh) && successor->state_expanded == false && Is_valid_3D(successor,map,x_size,y_size,collision_thresh)) 
                {
                    if (successor->g_cost > current->g_cost + cost)
                    {                 
                        // cout<<"Valid Successor"<<endl;
                        
                        successor->h_cost = weight*OPEN_STATES_3D_temp[GETMAPINDEX(successor->x,successor->y,x_size,y_size)]->g_cost;
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
    cout<< OPEN.size()<<endl;
    cout<<OPEN_STATES_3D.size()<<endl;
    if (policy == false) 
    {
        get_path(current, path_3D);
        // shift_coordinates(path_3D,x_size,y_size);
        
    }


}




int main()
{
    
    pair <array<int,NUMOFDIRS_3D>,array<int,NUMOFDIRS_3D>> delta = get_dx_dy(0);
    for (int i = 0;i<NUMOFDIRS_3D;i++)
    {
        cout<< delta.first[i] << "," <<delta.second[i] <<endl;
    }
    unordered_map <int , state*> OPEN_STATES;
    vector<state*> path;
    
    pair<int, int> map_size = get_dimensions("office.txt");
    double* map = create_map("office.txt",map_size);
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // A_star_2D(dX, dY, 16, 6, map, 100, map_size.first, map_size.second, 3, 4, OPEN_STATES, path, false, 1, true, false);
    // cout<<"States expanded: "<<OPEN_STATES.size()<<endl;
    
    // ofstream myfile ("A*.txt");
    // if (myfile.is_open())
    // {
    //     for (int i =0;i<path.size();i++)
    //     {
    //         myfile << path[i]->x <<","<< path[i]->y<<endl;
    //     }
    //     myfile.close();
    // }
    // else cout << "Unable to open file";

    ofstream myfile_ ("office_map.txt");
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
    
    
    int dTheta_3D[NUMOFDIRS_3D] = {45, 0, 315};
    unordered_map <int , state*> OPEN_STATES_3D;
    vector<state*> path_3D;
// 16,6, 0, 6,16, 0,
    A_star_3D(dTheta_3D, 15,4, 0, 3,16, 0, map, 150, map_size.first, map_size.second, 8,  OPEN_STATES_3D, path_3D, false, 1);
    cout<<path_3D.size()<<endl;

    ofstream myfile ("office_3d_path.txt");
    if (myfile.is_open())
    {
        for (int i =0;i<path_3D.size();i++)
        {
            myfile << path_3D[i]->x <<","<< path_3D[i]->y << "," << path_3D[i]->theta<<endl;
        }
        myfile.close();
    }
    else cout << "Unable to open file";

   
    
    // int count = 0;
    // for (int i=0;i<map_size.first;i++)
    // {
    //     for (int j = 0;j < map_size.second;j++)
    //     {
    //         for (int k=0;k<8;k++)
    //         {
    //             if (OPEN_STATES_3D.count(GETMAPINDEX_3D(i,j,k,map_size.first, map_size.second, 8)) == 0)
    //             {
    //                 cout<< i <<","<<j<<","<<k<<endl;
    //                 count++;
    //             }
    //         }
    //     }
    // }
    // cout<<count<<endl;
    return 0;
}


