cmake_minimum_required(VERSION 3.8)
project(crazyflie_sim)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# # Install launch and config files.
# install(DIRECTORY
#   config
#   data
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_flake8.py
    test/test_pep257.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()
