////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// count_islands
///
/// Source File which contains count_islands function implementations
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "count_islands.h"

using namespace std;
using namespace Eigen;

pair<int,int> islandCounter::worldToMatrix(const double x_world, const double y_world)
{
    pair<int, int> out;
    out.first = (int)((x_world - X_MIN) / RESOLUTION);
    out.second = (int)((y_world - Y_MIN) / RESOLUTION);

    return out;
}

MatrixXd islandCounter::toggleMatrix(MatrixXd& submatrix)
{
    MatrixXd temp = MatrixXd::Ones(submatrix.rows(), submatrix.cols()) - submatrix;
    return temp;
}

void islandCounter::setGeometry(input* data)
{
    // Again, we assume the data input is a structure as:
    // struct input {
    //      int n;
    //      vector<vector<float>> points;
    // };

    // compute matrix size
    auto rows = (int)((X_MAX - X_MIN)/RESOLUTION);
    auto cols = (int)((Y_MAX - Y_MIN)/RESOLUTION);

    // initialize matrix as all zeros (all water)
    matrix = MatrixXd::Zero(rows, cols);

    // Insert squares
    for(const auto& square : data->points){
        // get the current square indeces.
        // Recall: idx_start = pair<int,int> where;
        //  idx_start.first = x_idx;
        //  idx_start.second = y_idx;
        auto idx_start = worldToMatrix(square[0], square[1]);
        auto idx_end = worldToMatrix(square[2], square[3]);

        // since we need the delta, compute the number of
        // rows and cols the square takes in matrix
        int n_rows = idx_end.first - idx_start.first;
        int n_cols = idx_end.second - idx_start.second;

        // toggle the submatrix with new square
        // for beauty, define a reference:
        MatrixXd temp_mat = matrix.block(idx_start.first, idx_start.second, n_rows, n_cols);
        matrix.block(idx_start.first, idx_start.second, n_rows, n_cols) = toggleMatrix(temp_mat);
    }
}

/// This is a simple recursive Depth-First-Search (DFS) implementation.
/// For simplicity, since no rectangles are skew, we only move top-down, left-right.
/// Also, we keep track of what elements have been explored, to avoid loops.
/// The search only goes in land nodes.
void islandCounter::dfs(const int i, const int j, const MatrixXd& matrix)
{
    visited[i][j] = true;

    if(i+1<matrix.rows() && matrix(i+1, j)==1 && visited[i+1][j]==false){
        dfs(i+1, j, matrix);
    }
    if(j+1<matrix.cols() && matrix(i, j+1)==1 && visited[i][j+1]==false){
        dfs(i, j+1, matrix);
    }
    if(i-1>=0 && matrix(i-1, j)==1 && visited[i-1][j]==false){
        dfs(i-1, j, matrix);
    }
    if(j-1>=0 && matrix(i, j-1)==1 && visited[i][j-1]==false){
        dfs(i, j-1, matrix);
    }
}

/// This is the main function which returns the number of islands that exist
int islandCounter::countIslands(input* data)
{
    // First, we check if data contains squares. If no data,
    // there are no islands.
    if(data->points.empty()){
        return 0;
    }

    // First, we need to set the geometry of the problem,
    // which populates the matrix.
    setGeometry(data);

//    cout << matrix << endl;

    // Initialize island counter
    int islands{0};

    // Initialize array of visited indeces with same size than matrix.
    visited = std::vector< std::vector<bool> >(matrix.rows(), vector<bool>(matrix.cols(), false));

    // loop through matrix to explore all elements
    for(int i=0; i<matrix.rows(); i++){
        for(int j=0; j<matrix.cols(); j++){

            // If we find land, we explore the graph using dfs.
            // which will eventually make an island and mark
            // all the connected land nodes as visited so we
            // skip them next search.
            if(matrix(i, j) == 1 && visited[i][j]==false){

                // We call dfs, which looks for conected land
                // and keeps exploring connected land children recursively.
                dfs(i, j, matrix);

                // when we leave this function, we have found and
                // visited all land nodes. so we have an island
                islands++;

            }
        }
    }

    return islands;
}


// function to add valid children to queue
void islandCounter::addChildren(const int px, const int py, const std::vector<std::vector<int> > &grid) {
    if( visited[px][py] ) return;

    visited[px][py] = true;

    // search to the right
    if( px + 1 < IDX_MAX && grid[px + 1][py] == 1 && visited[px+1][py] == false ) {
        open_nodes.push_front( make_pair(px+1, py) );
    }
    // search to the left
    if( px - 1 >= 0 && grid[px - 1][py] == 1 && visited[px - 1][py] == false) {
        open_nodes.push_front( make_pair(px-1, py) );
    }
    // search top
    if( py - 1 >= 0 && grid[px][py - 1] == 1 && visited[px][py - 1] == false) {
        open_nodes.push_front( make_pair(px, py-1) );
    }
    // search below
    if( py + 1 < IDY_MAX && grid[px][py + 1] == 1 && visited[px][py+1] == false ) {
        open_nodes.push_front( make_pair(px, py+1) );
    }
}

// This is the main function using queue
int islandCounter::countIslands(const std::vector<std::vector<int> >& grid) {

    if( grid.empty() ) return 0;

    // Get extrema
    IDX_MAX = (int)(grid.size());
    IDY_MAX = (int)(grid[0].size());

    // initialize output and visited vector
    int n_islands{0};
    visited = std::vector< std::vector<bool> >(IDX_MAX, std::vector<bool>(IDY_MAX, false));

    // for all nodes
    for( size_t idx = 0; idx < IDX_MAX; idx++) {
        for( size_t idy = 0; idy < IDY_MAX; idy++) {

            // if find a new root:
            if( grid[idx][idy] == 1 && visited[idx][idy] == false ) {

                open_nodes.push_front( std::make_pair(idx, idy) );

                // keep exploring valid children and mark them as visited
                while( !open_nodes.empty() ) {

                    const std::pair<int, int> node_idx = open_nodes.front();
                    open_nodes.pop_front();
                    const int& px = node_idx.first;
                    const int& py = node_idx.second;

                    addChildren(px, py, grid);
                }

                n_islands++;

            }
        }
    }

    return n_islands;
}
