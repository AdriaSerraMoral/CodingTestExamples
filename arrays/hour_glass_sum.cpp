#include <bits/stdc++.h>

using namespace std;

// Create the function that returns true and assigns the hourglass sum (if it exist) or false if it does not exist because indeces are out of bounds
bool getHourGlassSum(int& hourglass_sum,const int row, const int col, const vector<vector<int>>& arr){
    // we know it is size 6 but make more general

    // erase current sum, just in case
    hourglass_sum = 0;

    // get size of total matrix
    const int n_rows = arr.size();
    const int n_cols = arr[0].size();

    // firt, check if any of the indeces of hour glass element is out of bounds.
    // if it is, the hourglass is invalid and return empty array.
    if((row+2 >= n_rows) || (col+2 >= n_cols)){
        return false;
    }

    for(int i=row; i<=row+2; i++){
        for(int j=col; j<=col+2; j++){

            // grab the current element
            int num = arr[i][j];

            // Check if we need to add or skip the element
            if((i==row+1) && ((j==col) || (j==col+2))){
                continue;
            } else {
                hourglass_sum += num;
            }
        }
    }

    cout << " " << endl;

    return true;

}

// Complete the hourglassSum function below.
int hourglassSum(vector<vector<int>> arr) {

    // initialize larger sum (assume the array exists)
    int largest_hourglas_sum = -999;

    for(int i=0; i<arr.size(); i++){
        for(int j=0; j<arr[0].size(); j++){

            int sum_value;

            if(getHourGlassSum(sum_value, i, j, arr)){
                // value exists, compare to current max
                if(sum_value > largest_hourglas_sum){
                    largest_hourglas_sum = sum_value;
                }
            }
        }
    }

    return largest_hourglas_sum;

}

int main()
{
    ofstream fout(getenv("OUTPUT_PATH"));

    vector<vector<int>> arr(6);
    for (int i = 0; i < 6; i++) {
        arr[i].resize(6);

        for (int j = 0; j < 6; j++) {
            cin >> arr[i][j];
        }

        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }

    int result = hourglassSum(arr);

    fout << result << "\n";

    fout.close();

    return 0;
}
