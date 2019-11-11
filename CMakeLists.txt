cmake_minimum_required (VERSION 3.5)

###############################################################################
################## Baseline CMAKE Settings
###############################################################################
if(DEFINED ENV{CodingTestExamples_DIR} AND NOT DEFINED ${CodingTestExamples_DIR})
    set(CodingTestExamples_DIR $ENV{CodingTestExamples_DIR})
else()
    set(CodingTestExamples_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# Project
project(CodingTestExamples CXX C)


###############################################################################
#### Compiler Settings
###############################################################################

# Cpp compiler version
set(CMAKE_CXX_STANDARD 14)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

###############################################################################
### Components / 3rd party software that can be turned ON/OFF across platforms.
###############################################################################


###############################################################################
################## Core include directories and definitions
###############################################################################
include_directories(${CodingTestExamples_DIR})
include_directories(${CMAKE_BINARY_DIR})
include_directories(nodelab)

#Additional Libraries
list(APPEND THIRD_PARTY_LIBS pthread)


##############################################################################
################## 3rd Party Packages.
##############################################################################

#### Eigen3
FIND_PACKAGE( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR} )
add_definitions(-DHAVE_EIGEN)
list(APPEND THIRD_PARTY_LIBS ${EIGEN3_LIBS} )

add_executable(
	StringToBinaryTree BinaryTree/StringToBinaryTree.cpp
)

add_executable(
        VerifyPreorder BinaryTree/verify_preorder.cpp
)

add_executable(
    CountIslands    CountIslands/count_islands.cpp  CountIslands/count_island_example.cpp
)

add_executable(
    HourGlassSum    arrays/hour_glass_sum.cpp
)

add_executable(
    MinSwaps2    arrays/min_swaps_2.cpp
)

add_executable(
    RangeSumQuery    arrays/range_sum_query.cpp
)

add_executable(
    MergeIntervals    arrays/merge_intervals.cpp
)

add_executable(
    MaxSubarraySum    arrays/max_subarray.cpp
)

add_executable(
    CountInversions    arrays/count_inversions.cpp
)

add_executable(
    BalancedBrackets    StacksAndQueues/balancedbrackets.cpp
)

add_executable(
    LaneDriving     SingleLaneDriving/single_lane_driving.cpp
)

add_executable(
    CoinChange      CoinChange/coin_change.cpp
)

add_executable(
    MinWindowSubstring  MinWindowSubstring/min_window_substring.cpp
)

add_executable(
    BestTimeBuyStock  BestTimeBuyStock/best_time_buy_stock.cpp
)

add_executable(
    TaskScheduler   TaskScheduler/task_scheduler.cpp
)


# Target 3rd party libraries
#target_link_libraries(CodingTestExamples ${THIRD_PARTY_LIBS})








