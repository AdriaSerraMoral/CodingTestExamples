/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  Given a char array representing tasks CPU need to do. It contains capital letters A to Z
//  where different letters represent different tasks. Tasks could be done without original
//  order. Each task could be done in one interval. For each interval, CPU could finish one
//  task or just be idle.
//
//  However, there is a non-negative cooling interval n that means between two same tasks,
//  there must be at least n intervals that CPU are doing different tasks or just be idle.
//
//  You need to return the least number of intervals the CPU will take to finish all the
//  given tasks.
//
//  (e.g., Input: tasks = ["A","A","A","B","B","B"], n = 2; Output = 8 )
//      Explanation: A->B->idle ->A -> B -> idle -> A -> B
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <unordered_set>

int leastInterval( const std::vector<char>& tasks, int n ) {
    std::vector< std::unordered_set<char> > vTasksDifferent;

    for( size_t idx=0, idx_end=tasks.size(); idx < idx_end; idx++) {
        const char& c_idx = tasks.at(idx);

        if( vTasksDifferent.empty() ){
            std::unordered_set<char> s_idx;
            s_idx.insert(c_idx);
            vTasksDifferent.push_back( s_idx );
            continue;
        }

        int jdx {0};
        while( true ) {
            if(vTasksDifferent.at(jdx).count(c_idx) > 0) {
                jdx++;

                if (jdx >= vTasksDifferent.size() ){
                    std::unordered_set<char> s_idx;
                    s_idx.insert(c_idx);
                    vTasksDifferent.push_back( s_idx );
                    break;
                }

            } else {
                vTasksDifferent.at(jdx).insert(c_idx);
                break;
            }

        }

    }

    int intervals {0};
    intervals += vTasksDifferent.size() - 1;

    for( size_t jdx = 0, jdx_end = vTasksDifferent.size(); jdx < jdx_end; jdx++ ){
        intervals += vTasksDifferent.at(jdx).size();
    }

    return intervals;
}

int main(int argc, char *argv[])
{
    int n {2};
    std::vector<char> tasks {'A', 'B', 'C', 'A', 'C', 'D', 'A', 'B'};

    std::cout << "We need " << leastInterval(tasks, n) << " CPU intervals " << std::endl;

    return 0;
}
