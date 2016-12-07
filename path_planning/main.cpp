//
//  main.cpp
//  path_planning
//
//  Created by Antoine Laurens on 20.11.16.
//  Copyright © 2016 Antoine Laurens. All rights reserved.
//


#include <iostream>
#include "path_planning_gr14.h"
#include <fstream>
using namespace std;


int main(int argc, const char * argv[]) {
    vector<array<float,2> >path;
    init_grid();
    int start = 0;
    int end = 425;
    a_star(start, end);
    path = generate_path(start,end);
    const char *path_file="/Users/antoinelaurens/Desktop/path_planning/path_planning/example.txt";
    ofstream myfile(path_file);
    for (int i = 0;i<path.size();i++)
    {
        myfile << path[i][0]<<" "<<path[i][1]<<";"<<endl;
    }

    return 0;
}
