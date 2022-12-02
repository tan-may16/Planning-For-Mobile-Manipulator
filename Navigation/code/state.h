#include <bits/stdc++.h>
class state {
    public:
    int x;
    int y;
    int theta;
    double g_cost;
    double h_cost;
    double f_cost;
    bool state_expanded;
    state * parent;
    state(){}
    state(int robotposx, int robotposy, int theta)
    {
        this->x = robotposx;
        this->y = robotposy;
        this->theta = theta;
        parent = NULL;
        this->g_cost = INT_MAX;
        this->h_cost = 0;
        this->f_cost = INT_MAX;
        state_expanded = false;
    }
    
};
