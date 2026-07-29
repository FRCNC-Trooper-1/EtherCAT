# Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
#
# Deliberately dependency-free. A controller that can only be tested after
# fetching a framework from the internet cannot be tested on a shop-floor
# machine with no network. Add a framework later if the suite outgrows this.

add_executable(test_cyclic_task test_cyclic_task.cpp)
target_link_libraries(test_cyclic_task PRIVATE frcnc_rt frcnc_flags)
add_test(NAME cyclic_task COMMAND test_cyclic_task)

add_executable(test_cia402 test_cia402.cpp)
target_link_libraries(test_cia402 PRIVATE frcnc_drive frcnc_flags)
add_test(NAME cia402 COMMAND test_cia402)

add_executable(test_profile test_profile.cpp)
target_link_libraries(test_profile PRIVATE frcnc_motion frcnc_flags)
add_test(NAME profile COMMAND test_profile)
