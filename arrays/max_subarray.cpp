/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  Given an integer array nums, find the contiguous subarray (containing at least one number)
//  which has the largest sum and return its sum.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>

class MaxSubarray {
public:
    int maxSubarray( const std::vector<int>& nums) {
        int max_sum {INT_MIN};
        int sum_idx {0};
        max_sum_array.reserve(nums.size());
        auto it_max_begin = nums.begin();
        auto it_max_end = nums.begin();

        for( size_t idx = 0, idx_end = nums.size(); idx < idx_end; idx++) {
            if( nums.at(idx) > max_sum ) {
                // if the number is already greater than our max sum, just make that the
                // begining
                max_sum = nums.at(idx);
                it_max_begin = nums.begin() + idx;
                sum_idx = max_sum;
                continue;
            }

            // add next number
            sum_idx += nums.at(idx);

            if( sum_idx > max_sum ) {
                max_sum = sum_idx;
                it_max_end = nums.begin() + idx;
            }
        }

        max_sum_array = std::vector<int>(it_max_begin, it_max_end+1);

        return max_sum;
    }

    std::vector<int> max_sum_array;

};

int main(int argc, char *argv[])
{
    std::vector<int> nums {-2, 1, -3, 4, -1, 2, 1, -5, 4};

    MaxSubarray ms;

    int max_sum = ms.maxSubarray(nums);

    std::cout << "Max sum subarray is:  " << max_sum << std::endl;

}
