
ADD_EXECUTABLE( simple-example "SimpleExample.cpp" )
TARGET_LINK_LIBRARIES( simple-example ${Boost_LIBRARIES} )

ADD_EXECUTABLE( frameCtrlExample "FrameCtrlExample.cpp" )
TARGET_LINK_LIBRARIES( frameCtrlExample ${Boost_LIBRARIES} )

ADD_EXECUTABLE( clientExample "clientExample.cpp" )
TARGET_LINK_LIBRARIES( clientExample ${Boost_LIBRARIES} )
