/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  If we can swap adjacent elements, count the number of swaps to return a sorted array
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>

/**
 * @brief countInversions
 * @param arr
 * @return
 *
 *  This is n log n ? best case is o(n), worst case it is n^2?
 *
 *  Could try to make faster by running from beginning to end and from
 *  end to beginning simultaneously. so we do 0, 1, or 2 swaps per iteration.
 *
 */
long countInversions( std::vector<int> arr ) {
    if( arr.empty() ) return 0;

    if( arr.size() == 1 ) return 0;

    long total_swaps {0};

    int idx = 0;
    int jdx = 1;

    while ( jdx < arr.size() ){
        if( arr.at(jdx) < arr.at(idx) ){
            int val_jdx = arr.at(jdx);
            arr.at(jdx) = arr.at(idx);
            arr.at(idx) = val_jdx;

            idx--;
            if( idx < 0) {
                idx = 0;
            }
            jdx = idx+1;

            total_swaps++;

        } else {
            idx++;
            jdx++;
        }
    }

    return total_swaps;
}

int main(int argc, char *argv[])
{

    std::vector<int> arr {1, 1, 1, 2, 2};
//    std::vector<int> arr {2, 1, 3, 1, 2};

    std::cout << "Total number of swaps: " << countInversions(arr) << std::endl;

    return 0;
}
