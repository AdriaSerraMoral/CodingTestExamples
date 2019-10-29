#include <bits/stdc++.h>
#include <map>

using namespace std;

vector<string> split_string(string);

// Function to score an array (our heuristic)
int scoreArray( vector<int> arr) {

    int score {0};

    for( size_t idx = 0, idx_end = arr.size(); idx < idx_end; idx++ ) {
        // get value at idx
        const int& val = arr.at(idx);

        // the goal idx is val - 1
        const int goal_idx = val - 1;

        // add error.
        score += abs( (int)idx - goal_idx );
    }

    return score;
}

// Complete the minimumSwaps function below.
int minimumSwaps(vector<int> arr) {

     // Store current index that we are swapping from, and the total number of elements
    // so we can exit while loop once it is all sorted
    size_t idx {0};
    size_t arr_size = arr.size();
    size_t idx_end = arr_size - 1;

    // I will use a map to keep track of the minimum score automatically,
    // the key is the minimum score and the value is the best array
    map<int, vector<int>> swap_map;

    // number of swaps counter
    int swap_counter {0};

    // I will iterate using a while loop since I only want to move on to the next
    // index once I have the right value in the current index, which may not happen
    // in just one swap.
    // while we do not have a solution:
    while(true) {

        // if our current idx is at the end of array, nothing to swap, we are done
        if( idx == idx_end ) return swap_counter;

        // if the value at the current index is in order, do not do a swap
        if( idx == arr.at(idx) - 1 ) {
            // move idx to the right and check that sub-array.
            idx++;
            continue;
        }

        // at every start, we clear the map
        swap_map.clear();

        // we need a swap, check options from idx to end since elements
        // left of idx are in order
        for( size_t jdx = idx+1; jdx < arr_size; jdx++ ){

            // swap idx and jdx elements and record score
            vector<int> array_idx = arr;
            swap( array_idx[idx], array_idx[jdx] );
            int score = scoreArray( array_idx );

            // if score is 0, this is the goal state, just add swap and return
            if( score == 0 ) {
                swap_counter++;
                return swap_counter;
            }

            swap_map[ score ] = array_idx;

        }

        // we have populated swap_map, take the best score (the front)
        arr = swap_map.begin()->second;

        // add to counter
        swap_counter++;

    }

    return swap_counter;

}

int main()
{
    ofstream fout(getenv("OUTPUT_PATH"));

    vector<int> arr{ 4, 3, 2, 1 };

    int res = minimumSwaps(arr);

    fout << res << "\n";

    fout.close();

    return 0;
}

vector<string> split_string(string input_string) {
    string::iterator new_end = unique(input_string.begin(), input_string.end(), [] (const char &x, const char &y) {
        return x == y and x == ' ';
    });

    input_string.erase(new_end, input_string.end());

    while (input_string[input_string.length() - 1] == ' ') {
        input_string.pop_back();
    }

    vector<string> splits;
    char delimiter = ' ';

    size_t i = 0;
    size_t pos = input_string.find(delimiter);

    while (pos != string::npos) {
        splits.push_back(input_string.substr(i, pos - i));

        i = pos + 1;
        pos = input_string.find(delimiter, i);
    }

    splits.push_back(input_string.substr(i, min(pos, input_string.length()) - i + 1));

    return splits;
}
