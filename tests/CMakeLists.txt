# Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
#
# Deliberately dependency-free. A controller that can only be tested after
# fetching a framework from the internet cannot be tested on a shop-floor
# machine with no network. Add a framework later if the suite outgrows this.

add_executable(test_cyclic_task test_cyclic_task.cpp)
target_link_libraries(test_cyclic_task PRIVATE frcnc_rt frcnc_flags)
add_test(NAME cyclic_task COMMAND test_cyclic_task)
