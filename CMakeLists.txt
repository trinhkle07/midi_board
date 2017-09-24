cmake_minimum_required(VERSION 2.8.3)
project(midi_board)

find_package(catkin REQUIRED)
find_package(hmi_cmake REQUIRED)


catkin_python_setup()

catkin_package()

include(CodeFormat)
format_code_target()

catkin_install_python(PROGRAMS scripts/midi_board scripts/virtual_midi_board
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)

  catkin_add_nosetests(test)

  include(CodeAnalysis)
  lint_py(MODULE_DIR src/midi_board scripts)
  lint_cmake(FILES CMakeLists.txt)

  include(CodeCoverage)
  python_coverage_add_target()

endif()
