add_executable(test_obs_builder       test_obs_builder.cpp)
target_link_libraries(test_obs_builder PRIVATE qmini_user)
add_test(NAME obs_builder COMMAND test_obs_builder)

add_executable(test_math               test_math.cpp)
target_link_libraries(test_math PRIVATE qmini_user)
add_test(NAME math COMMAND test_math)

add_executable(test_leg_ik             test_leg_ik.cpp)
target_link_libraries(test_leg_ik PRIVATE qmini_user)
add_test(NAME leg_ik COMMAND test_leg_ik)

add_executable(test_sim_loop           test_sim_loop.cpp)
target_link_libraries(test_sim_loop PRIVATE qmini_user)
# Run sim loop from a tempdir we provision with the right config.yaml.
add_test(NAME sim_loop
    COMMAND ${CMAKE_COMMAND} -E chdir
            ${CMAKE_CURRENT_SOURCE_DIR}/fixtures
            $<TARGET_FILE:test_sim_loop>)

if (BACKEND STREQUAL "mujoco")
    add_executable(test_mujoco_loop    test_mujoco_loop.cpp)
    target_link_libraries(test_mujoco_loop PRIVATE qmini_user)
    # MuJoCo loader needs sim_assets/q1_sim.mjcf in CWD; fixtures/ has
    # config.yaml + a symlink to sim_assets/.
    add_test(NAME mujoco_loop
        COMMAND ${CMAKE_COMMAND} -E chdir
                ${CMAKE_CURRENT_SOURCE_DIR}/fixtures
                $<TARGET_FILE:test_mujoco_loop>)
endif()
