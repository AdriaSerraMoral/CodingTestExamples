/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  Given a string S and a string T, find the minimum window in S which will contain all the
//  characters in T in complexity O(n).
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <queue>
#include <vector>
#include <climits>
#include <algorithm>

class MinWindowSubstring {
public:

    std::string minWindow( const std::string& s, const std::string& t) {

        if ( s.empty() || t.empty() ) return "";

        std::string output = "";
        std::string output_idx = "";
        auto it_start = s.begin();

//        std::deque<std::string> sq;

        // first, while we do not have one of our substring letters, increase
        // it start
        while( std::find(t.begin(), t.end(), *it_start) == t.end() ) {
            it_start++;
        }

        // create end iterator at same start place
        auto it_end = it_start;

        // Initialize our word
        output_idx += *it_start;


        while( it_end != s.end() && it_start != s.end() ) {

            // first, while we do not have all the words of t, increase last iterator
            while( !hasT( output_idx, t ) ) {
                it_end++;
                if( it_end != s.end() ){
                    output_idx += * it_end;
                } else {
                    return output;
                }
            }

            // this has all the values in T
            if( output.empty() ){
                output = output_idx;
            } else {
                if( output.size() > output_idx.size() ){
                    output = output_idx;
                }
            }

            // start decreasing from left while we have all values
            while( hasT( output_idx, t) ) {
                if( output.size() > output_idx.size() ){
                    output = output_idx;
                }
                it_start++;
                output_idx = std::string(it_start, it_end+1);
            }

        }

        return output;

    }


private:

    bool hasT(const std::string& s_tmp, const std::string& t) {
        std::string t_tmp = t;

        for( const auto& c : s_tmp ){
            auto it_idx = std::find( t_tmp.begin(), t_tmp.end(), c );
            if( it_idx != t_tmp.end() ){
                t_tmp.erase(it_idx);
            }
        }

        return t_tmp.empty();
    }

};

int main(int argc, char *argv[])
{

    std::string s = "ADOBECODEBANC";
    std::string T = "ABC";

    MinWindowSubstring mws;

    std::string output = mws.minWindow(s, T);

    std::cout << "Our output string is:  " << output << "  \n";

    return 0;
}

