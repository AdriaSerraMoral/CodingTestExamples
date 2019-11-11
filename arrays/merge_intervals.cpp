/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  Given a collection of intervals, merge all overlapping intervals.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>

class MergeIntervals {
public:

    static std::vector< std::vector<int> > merge( std::vector< std::vector<int> >& intervals ) {

        if( intervals.empty() ) {
            return std::vector< std::vector<int> >();
        }
        if( intervals.size() == 1) {
            return intervals;
        }

        // first, I put intervals as vector of pairs, and I order it
        std::vector< std::pair< int, int> > vPairs(intervals.size());

        for( size_t idx = 0, idx_end = intervals.size(); idx<idx_end; idx++){

            vPairs.at(idx) = std::make_pair(intervals.at(idx)[0], intervals.at(idx)[1] );

        }

        std::sort( vPairs.begin(), vPairs.end() );

        std::vector< std::vector<int> > output;
        output.reserve( intervals.size() );

        auto it_idx = vPairs.begin();
        auto it_jdx = it_idx;
        it_jdx++;

        int val_min = it_idx->first;
        int val_max = it_idx->second;

        while( true ){

            if( it_jdx->first > val_max ) {
                // add to output
                output.push_back( std::vector<int>{val_min, val_max} );
                // update
                val_min = it_jdx->first;
                val_max = it_jdx->second;

                it_jdx++;
                if( it_jdx == vPairs.end() ){
                    output.push_back( std::vector<int>{val_min, val_max} );
                    return output;
                }

                continue;
            }

            if( it_jdx->second > val_max ) {
                val_max = it_jdx->second;

                // move jdx iterator
                it_jdx++;

                if( it_jdx == vPairs.end() ) {
                    // add to output and end
                    output.push_back( std::vector<int>{ val_min, val_max} );

                    return output;
                }

                continue;

            }

            it_jdx++;
            if( it_jdx == vPairs.end() ) {
                // add to output and end
                output.push_back( std::vector<int>{ val_min, val_max} );

                return output;
            }

        }

        return output;

    }

private:


};


int main(int argc, char *argv[])
{

//    std::vector< std::vector<int> > vm { {1, 9}, {2, 5}, {19, 20}, {10, 11}, {12, 20}, {0, 3}, {0, 1}, {0, 2} };
//    std::vector< std::vector<int> > vm { {1, 3}, {2, 6}, {8, 10}, {15, 18} };
    std::vector< std::vector<int> > vm { {1, 4}, {4, 5} };

    std::vector< std::vector<int>> output = MergeIntervals::merge(vm);

    std::cout << "[";
    for( const auto v : output ) {
        std::cout << "(" << v[0] << "," << v[1] << "),";
    }
    std::cout << "]" << std::endl;

    return 0;
}
