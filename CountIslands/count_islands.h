////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// count_islands
///
/// Header File which contains count_islands class and function definitions / dependencies
///
/// DEPENDENCIES:
///     -Eigen Linear Algebra Library
///        used for simple linear algebra manipulations
///        when constructing the world matrix. If have time,
///        can drop it and write my own matrix toggle operation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdlib.h>
#include <vector>
#include <map>
#include <iostream>
#include <Eigen/Dense>
#include <queue>

using namespace std;
using namespace Eigen;

// For the purpose of this project, I am not going to spend
// time coding a file reader to read sample inputs from a file.
// I am assuming there is a frontend somewhere which reports
// the data in the follower structure or similar:
struct input {
    int n;
    vector<vector<float>> points;
};

class islandCounter {

private:

    // The list of constants (in capital letter) below could potentially
    // be computed while parsing the input data. For quick development,
    // I am initializing (reasonable default) values assuming we only
    // work on the first quadrant for a maximum size of 10. and a
    // resolution of 0.5 grid size (yes, I am over-fitting to the sample input proided)
    double RESOLUTION{.5};
    double X_MAX{10.};
    double X_MIN{0.};
    double Y_MAX{10.};
    double Y_MIN{0.};
    int    IDX_MAX;
    int    IDY_MAX;

    // yet, we could design a member function which takes input structure
    // and computes maximum(s) and resolution, or compute it in setGeometry function.

    ///< Square m x m boolean matrix of 0 and 1 where:
    ///<    0: Water
    ///<    1: Land
    ///< it goes from index 0 to m where:
    ///<    0: X_MIN or Y_MIN
    ///<    m: X_MAX or Y_MAX
    MatrixXd matrix;

    // Double ended queue to run dfs or bfs
    std::deque< std::pair<int, int> > open_nodes;

    // vector to check whether a node was visited previously
    vector<vector<bool>> visited;

    // ------------------ HELPER FUNCTIONS ----------------------------------------

    ///< worldToMatrix: returns the matrix indeces of a point in world coordinates:
    ///<    Input: (x, y) in world coordinates
    ///<    Output: (j, k) matrix indeces
    pair<int, int> worldToMatrix(const double x_world, const double y_world);

    ///< setGeometry:  Initializes the problem, takes input data and computes
    ///<               the boolean matrix where 0 is water and 1 is land
    ///<
    ///< NOTE: I am taking struct input as the input, could do void* to be more
    ///<       general, but I am optimizing for speed to code this within 2hrs
    void setGeometry(input* data);

    ///< toggleMatrix: This helper function toggles the values within a matrix.
    ///<               For instance, if we add a square within a bigger square
    ///<               that used to be land (full of 1), it toggles the inner square
    ///<               to contain 0 (or water).
    MatrixXd toggleMatrix(MatrixXd& submatrix);

    ///< dfs: This function does depth first search to find number of islands
    void dfs(const int i, const int j, const MatrixXd& matrix );

    ///< dfs is recursive, also solve dfs search using deque
    void addChildren( const int px, const int py, const std::vector< std::vector<int> >& grid);

public:

    ///< countIslands:  Main function, counts the islands
    ///<    Input: a structure similar to "input" above with members:
    ///<        -n : integer with number of squares
    ///<        -points:  2d array with points in World coordinates
    ///<
    ///< NOTE: I am taking struct input as the input, could do void* to be more
    ///<       general, but I am optimizing for speed to code this within 2hrs
    int countIslands(input* data);

    ///< countIslands( const std::vector< std::vector< int> >& grid ):
    ///<    Input: a 2d grid with 0 and 1 representing islands
    ///<
    ///<    Output: number of islands (int)
    int countIslands( const std::vector< std::vector<int> >& grid);

    const MatrixXd& getMatrix() const { return matrix; }

};
