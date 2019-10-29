
#include <iostream>
#include "count_islands.h"


using namespace std;
using namespace Eigen;

int main(int argc, char** argv){

    input data;

    data.n = 14;

    data.points.clear();
    
    vector<float> temp_array;
    temp_array.push_back(1.0);
    temp_array.push_back(1.0);
    temp_array.push_back(10.0);
    temp_array.push_back(6.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(1.5);
    temp_array.push_back(1.5);
    temp_array.push_back(6.0);
    temp_array.push_back(5.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(2.0);
    temp_array.push_back(2.0);
    temp_array.push_back(3.0);
    temp_array.push_back(3.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(2.0);
    temp_array.push_back(3.5);
    temp_array.push_back(3.0);
    temp_array.push_back(4.5);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(3.5);
    temp_array.push_back(2.0);
    temp_array.push_back(5.5);
    temp_array.push_back(4.5);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(4.0);
    temp_array.push_back(3.5);
    temp_array.push_back(5.0);
    temp_array.push_back(4.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(4.0);
    temp_array.push_back(2.5);
    temp_array.push_back(5.0);
    temp_array.push_back(3.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(7.0);
    temp_array.push_back(3.0);
    temp_array.push_back(9.5);
    temp_array.push_back(5.5);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(7.5);
    temp_array.push_back(4.0);
    temp_array.push_back(8.0);
    temp_array.push_back(5.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(8.5);
    temp_array.push_back(3.5);
    temp_array.push_back(9.0);
    temp_array.push_back(4.5);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(3.0);
    temp_array.push_back(7.0);
    temp_array.push_back(8.0);
    temp_array.push_back(10.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(5.0);
    temp_array.push_back(7.5);
    temp_array.push_back(7.5);
    temp_array.push_back(9.5);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(5.5);
    temp_array.push_back(8.0);
    temp_array.push_back(6.0);
    temp_array.push_back(9.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    temp_array.push_back(6.5);
    temp_array.push_back(8.0);
    temp_array.push_back(7.0);
    temp_array.push_back(9.0);
    data.points.push_back(temp_array);
    temp_array.clear();

    islandCounter island_counter;

    int islands = island_counter.countIslands(&data);

    cout << "islands are:  " << islands << endl;

    cout << " " << endl;

    cout << island_counter.getMatrix() << endl;

}
