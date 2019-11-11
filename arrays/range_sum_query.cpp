/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  Given an integer array nums, find the sum of the elements between indices i and j (i ≤ j),
//  inclusive.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <vector>

/**
 * @brief The NumArray class
 *
 *  As usual, one way is to keep vector in memory (or pointer), and use iterators for
 *  quick acess to data and in a loop, add all values O(n).
 *
 *  Another approach is to get faster speed by using more memory. we could:
 *      a) have an array of all possible combinations i->j where i <= j. and return array[i][j]
 *      b) have an array with sum_k, where sum[i] = sum[0] + sum[1] + ... + sum[i].
 *          Then, sum_ij = sum[j+1] - sum[i]
 */
class NumArray {
public:
    NumArray( const std::vector<int>& nums ) {
        this->nums = nums;

        sum_k = std::vector<int>(nums.size() + 1, 0);

        for( size_t idx = 0, idx_end = nums.size(); idx < idx_end; idx++ ){
            sum_k.at(idx+1) = sum_k.at(idx) + nums.at(idx);
        }

    }

    int sumRange( int i, int j) {
        it_begin = nums.begin() + i;
        it_end = nums.begin() + j + 1;
        int sum {0};

        while( it_begin != it_end ) {
            sum += *it_begin;

            it_begin++;
        }

        return sum;
    }

    int sumRangeFast( int i, int j ) {
        return (sum_k.at(j+1) - sum_k.at(i) );
    }

    void update(int i, int val) {
        // for faster version, we need to update sum_k from i forward.
        // get delta sum, and add that to all next indeces
        int delta_num = val - nums.at(i);

        // for slow version, just change value of nums internal
        nums.at(i) = val;

        // now, update sum_k
        for( size_t idx = i+1, idx_end = sum_k.size(); idx < idx_end; idx++) {
            sum_k.at(idx) += delta_num;
        }

    }

private:

    std::vector<int> nums;

    std::vector<int> sum_k;

    std::vector<int>::iterator it_begin;
    std::vector<int>::iterator it_end;

};

int main(int argc, char *argv[])
{

    std::vector<int> nums {-2, 0, 3, -5, 2, -1};
    int i {0};
    int j {5};

    NumArray* obj = new NumArray(nums);

    std::cout << "Sum from i = " << i << ", j = " << j << "  is:  " << obj->sumRange(i, j) << std::endl;
    std::cout << "Sum from i = " << i << ", j = " << j << "  is:  " << obj->sumRangeFast(i, j) << std::endl;

    obj->update( 3, 0 );

    std::cout << "Sum from i = " << i << ", j = " << j << "  is:  " << obj->sumRange(i, j) << std::endl;
    std::cout << "Sum from i = " << i << ", j = " << j << "  is:  " << obj->sumRangeFast(i, j) << std::endl;

    return 0;
}
