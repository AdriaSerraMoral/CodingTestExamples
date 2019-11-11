/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  Given a string of comma separated values, verify whether it is a correct preorder
//  traversal serialization of a binary tree. Find an algorithm without reconstructing the tree.
//
//  Each comma separated value in the string must be either an integer or a character '#'
//  representing null pointer.
//
//  You may assume that the input format is always valid, for example it could never contain
//  two consecutive commas such as "1,,3".
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>

bool isValidSerialization(const std::string& preorder) {

    int available_childs {1};

    for( const char& c : preorder ){

        std::string s(1, c);

        if( s == "," )  continue;

        if( s == "#") {
            available_childs--;
        } else {
            available_childs++;
        }
    }

    return available_childs == 0;

}

int main(int argc, char *argv[])
{

    std::string input = "9,3,4,#,#,1,#,#,2,#,6,#,#";

    if( isValidSerialization(input) ){
        std::cout << "true" << std::endl;
    } else {
        std::cout << "false" << std::endl;
    }

    return 0;
}
