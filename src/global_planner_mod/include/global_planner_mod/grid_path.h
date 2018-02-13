
#ifndef _GRID_PATH_H
#define _GRID_PATH_H
#include<vector>
#include<global_planner_mod/traceback.h>

namespace global_planner_mod {

class GridPath : public Traceback {
    public:
        GridPath(PotentialCalculator* p_calc): Traceback(p_calc){}
        bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
};

} //end namespace global_planner_mod
#endif
