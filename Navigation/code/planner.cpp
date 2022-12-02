/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <bits/stdc++.h>
#include "planner.h"
#include <vector>
#include <stack>
#include <chrono>
using namespace std::chrono;
using namespace std;
/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
double heurastic_1 (int robotposeX, int robotposeY, int goalposeX, int goalposeY)
{
    //Euclidean h
    // double h = sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX)) + ((robotposeY-goalposeY)*(robotposeY-goalposeY)));

    //Best h
    int delta_x = robotposeX - goalposeX;
    int delta_y = robotposeY - goalposeY;
    double h =   sqrt(2)*MIN(abs(delta_x),abs(delta_y)) + ((MAX(abs(delta_x),abs(delta_y)))-(MIN(abs(delta_x),abs(delta_y))));
    // double h =   (abs(delta_x)+abs(delta_y)) + (1 - 2)*MIN(abs(delta_x),abs(delta_y));

    // double h = MAX(abs(robotposeX-goalposeX),abs(robotposeY-goalposeY));
    return h;
}
double heurastic_2 (int robotposeX, int robotposeY, int goalposeX, int goalposeY)
{
    //Euclidean h
    double h = MAX(abs(robotposeX-goalposeX),abs(robotposeY-goalposeY));
    return h;
}
pair<int,int> calculate_path_steps(int i, double* map, state * parent_current, stack <pair<int,int>> &robot_traj, int x_size, int y_size, int target_steps, double* target_traj, bool return_path = false)
{
    while (parent_current!= NULL)
    {
        pair<int, int> pos (parent_current->robotposx,parent_current->robotposy);
        robot_traj.push(pos);
        if (parent_current->parent!=NULL) parent_current = parent_current->parent;
        else break;
    }
    pair<int, int> cost (i, robot_traj.size());
    if (return_path == false)
    {
        while(!robot_traj.empty())
        {
            robot_traj.pop();
        }
    }
    return cost;

} 
pair<int,int> calculate_path_cost(int i, double* map, state * parent_current, stack <pair<int,int>> &robot_traj, int x_size, int y_size, int target_steps, double* target_traj, bool return_path = false)
{
    int path_cost = 0;
    int cell_cost = 0;
    while (parent_current!= NULL)
    {
        if (parent_current->parent!=NULL) 
        {
            parent_current = parent_current->parent;
            path_cost = path_cost + map[(int)GETMAPINDEX(parent_current->robotposx,parent_current->robotposy,x_size,y_size)];
        }else break;
        
    }
    pair<int, int> cost (i, path_cost);
    return cost;

} 

auto cmp= [](const state* a, const state* b)
    {
        if (a->f_cost == b->f_cost)
        {
            return a->h_cost > b->h_cost;
        }
        return a->f_cost > b->f_cost;
    };
auto cmp_pair= [](const pair<int,int> a, const pair<int,int> b)
    {
        return a.second > b.second;
    };
int loop_count = 0;
stack <pair<int,int>> robot_traj;
priority_queue <pair<int,int>> target_g_cost;

stack <pair<int,int>> A_star_policy(int dX[],int dY[],int goalposeX, int goalposeY, double*map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY, int target_steps, double* target_traj, bool policy=true, int weight = 1)
{
    auto start_time = high_resolution_clock::now();
    int while_count = 0;
    priority_queue <state*,vector<state*>,decltype(cmp)> OPEN(cmp);
    unordered_map <int , state*> OPEN_STATES;
    
    state *goal = new state(goalposeX,goalposeY);
    state *successor = new state(robotposeX + 1,robotposeY + 1);
    state *current = new state(robotposeX,robotposeY);
    current->g_cost = 0;
    OPEN.push(current);
    OPEN_STATES.insert(make_pair(GETMAPINDEX(current->robotposx,current->robotposy,x_size,y_size),current));
    while (!OPEN.empty())
    {
        current = OPEN.top();
        if (policy == false && current->robotposx == goalposeX && current->robotposy == goalposeY)
        {
            break;
        }
        OPEN.pop();
        current->state_expanded = true;
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            if (OPEN_STATES.count(GETMAPINDEX(current->robotposx + dX[dir],current->robotposy + dY[dir],x_size,y_size))== 0)
            {
                successor = new state(current->robotposx + dX[dir],current->robotposy + dY[dir]);
            }
            else
            {
                successor = OPEN_STATES[GETMAPINDEX(current->robotposx + dX[dir],current->robotposy + dY[dir],x_size,y_size)];
                
            }
            if (successor->robotposx >= 1 && successor->robotposx <= x_size && successor->robotposy >= 1 && successor->robotposy <= y_size)
            {
                int cost = (int)map[GETMAPINDEX(successor->robotposx,successor->robotposy,x_size,y_size)];
                if ((cost >= 0) && (cost < collision_thresh) && successor->state_expanded == false) 
                {
                    if (successor->g_cost > current->g_cost +(int)map[GETMAPINDEX(successor->robotposx,successor->robotposy,x_size,y_size)])
                    {                 
                        if (weight == 1)
                        {
                            successor->h_cost = successor->h_cost = weight*heurastic_1(successor->robotposx,successor->robotposy,goalposeX,goalposeY);
                        
                        }
                        else
                        {
                            successor->h_cost = successor->h_cost = weight*heurastic_2(successor->robotposx,successor->robotposy,goalposeX,goalposeY);
                        }
                        successor->g_cost = current->g_cost +(int)map[GETMAPINDEX(successor->robotposx,successor->robotposy,x_size,y_size)];
                        successor->f_cost = successor->g_cost + successor->h_cost;
                        successor->parent = current;
                        if (OPEN_STATES.count(GETMAPINDEX(current->robotposx + dX[dir],current->robotposy + dY[dir],x_size,y_size))== 0)
                        {
                            OPEN.push(successor);
                        }
                        OPEN_STATES[GETMAPINDEX(successor->robotposx,successor->robotposy,x_size,y_size)] = successor;
                        
                    }
                }
            }
        }
        while_count++;

        if (while_count>10000000)
        {
            cout<<"While loop Cut"<<endl;
            cout<<"Planning failed"<<endl;
            break;
        }

    }
    int final_index = (int)GETMAPINDEX(robotposeX,robotposeY,x_size,y_size);
    state * final_state = OPEN_STATES[final_index];
    if (policy == true)
    {

    
        priority_queue <pair<int,int>,vector<pair<int,int>>,decltype(cmp_pair)> traj_cost(cmp_pair);
        for (int i = 0; i<target_steps - 1; i++)
        {
            current = OPEN_STATES[GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)];
            pair <int,int> cost_of_path = calculate_path_steps(i, map, current,robot_traj,x_size, y_size, target_steps, target_traj);
            int time_diff = i - cost_of_path.second;
            auto stop_time = high_resolution_clock::now();
            auto duration_time = duration_cast<seconds>(stop_time - start_time);
            if (time_diff > 0 + duration_time.count() )
            {
                if (weight == 1)
                {
                    int cell_cost = map[(int)GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)];
                    int path_cost = OPEN_STATES[(int)GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)]->g_cost;
                    int total_cost = path_cost + time_diff*cell_cost;
                    cost_of_path.second = total_cost;
                    traj_cost.push(cost_of_path);
                }
                else
                {
                    pair <int,int> actual_cost = calculate_path_cost(i, map, current,robot_traj,x_size, y_size, target_steps, target_traj);
                    int cell_cost = map[(int)GETMAPINDEX(target_traj[i-1],target_traj[i-1 + target_steps],x_size,y_size)];
                    
                    int total_cost = actual_cost.second + time_diff*cell_cost;
                    cost_of_path.second = total_cost;
                    traj_cost.push(cost_of_path);
                }
            }
            
        }
        if (traj_cost.empty())
        {
            return robot_traj;
        }
        while(!robot_traj.empty())
        {
            robot_traj.pop();
        }
        cout<<"Intersection at target_traj index ="<<traj_cost.top().first<<" ,"<<"Cost Estimate:"<<traj_cost.top().second<<endl;
        final_index = traj_cost.top().first;
        final_state = OPEN_STATES[(int)GETMAPINDEX(target_traj[final_index-1],target_traj[final_index-1 + target_steps],x_size,y_size)];
    
    }
    else
    {
        while(!robot_traj.empty())
        {
            robot_traj.pop();
        }
        if (while_count>10000000)
        {
            return robot_traj;
        }
        else
        {
            final_index = (int)GETMAPINDEX(current->robotposx ,current->robotposy,x_size,y_size);
            final_state = current;
        }

    }
    pair <int,int> cost_of_path = calculate_path_steps(final_index, map, final_state,robot_traj,x_size, y_size, target_steps, target_traj,true);
    return robot_traj;
}
pair <int, int> goalpos_cost;
static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};


    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    
    if (loop_count == 0)
    {
        stack <pair <int,int>> robot_traj = A_star_policy(dX,dY,goalposeX,goalposeY,map,collision_thresh,x_size,y_size,robotposeX,robotposeY,target_steps,target_traj, true , 1);
        int weight = 10;
        while(robot_traj.empty() && weight <= 90 )
        {
            cout<<"Weight = "<<weight<<endl;
            robot_traj = A_star_policy(dX,dY,goalposeX,goalposeY,map,collision_thresh,x_size,y_size,robotposeX,robotposeY,target_steps,target_traj, true , weight); 
            weight = 10+weight;  
        }
        if (weight >90 && robot_traj.empty())
        {
            robot_traj = A_star_policy(dX,dY,goalposeX,goalposeY,map,collision_thresh,x_size,y_size,robotposeX,robotposeY,target_steps,target_traj, false , weight);     
        }
        
    }
    if (robot_traj.empty())
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }
    else
    {
        action_ptr[0] = robot_traj.top().first;
        action_ptr[1] = robot_traj.top().second;
        robot_traj.pop();
    }
    loop_count++;

    return;
}
// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}