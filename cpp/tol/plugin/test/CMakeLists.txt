find_package(Boost COMPONENTS REQUIRED
        "system"
        "unit_test_framework"
        )
find_package (Threads)

add_executable(testsupgbrain test_SUPGBrain.cpp)
target_link_libraries(testsupgbrain ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_test(tol-testsupgbrain testsupgbrain)

#add_executable(testfakelightsensor testfakelightsensor.cpp)
#target_link_libraries(testfakelightsensor ${Boost_LIBRARIES} tolrobotcontrol)
#add_test(tol-testfakelightsensor testfakelightsensor)

#add_executable(testsupgbrainphototaxis testsupgbrainphototaxis.cpp)
#target_link_libraries(testsupgbrainphototaxis ${Boost_LIBRARIES} tolrobotcontrol)
#add_test(tol-testsupgbrainphototaxis testsupgbrainphototaxis)

add_executable(TestYAMLBodyParser test_YAMLBodyParser.cpp)
target_link_libraries(TestYAMLBodyParser ${Boost_LIBRARIES} tolrobotcontrol)
add_test(NAME tol-TestYAMLBodyParser
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND TestYAMLBodyParser
        )
