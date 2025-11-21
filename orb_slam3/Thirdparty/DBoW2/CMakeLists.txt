set(SRCS_DBOW2
    DBoW2/BowVector.cpp
    DBoW2/FORB.cpp      
    DBoW2/FeatureVector.cpp
    DBoW2/ScoringObject.cpp)

set(SRCS_DUTILS
    DUtils/Random.cpp
    DUtils/Timestamp.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})
