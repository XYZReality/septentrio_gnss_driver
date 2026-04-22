ament_add_gtest(test_string_utilities
  test_string_utilities.cpp
)

target_link_libraries(test_string_utilities
  ${library_name}
)

ament_add_gtest(test_parsing_utilities
  test_parsing_utilities.cpp
)

target_link_libraries(test_parsing_utilities
  ${library_name}
)

ament_add_gtest(test_ephemeris_parsers
  test_ephemeris_parsers.cpp
)

target_link_libraries(test_ephemeris_parsers
  ${library_name}
  ${${PROJECT_NAME}_TARGETS}
)
