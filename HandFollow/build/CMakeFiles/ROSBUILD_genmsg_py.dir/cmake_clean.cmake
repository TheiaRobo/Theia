FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/HandFollow/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/HandFollow/msg/__init__.py"
  "../src/HandFollow/msg/_PidParams.py"
  "../src/HandFollow/msg/_vw.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
