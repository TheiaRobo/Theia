FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/robot_messages/msg"
  "../msg_gen"
  "CMakeFiles/tests"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/tests.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
