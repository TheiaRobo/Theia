FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/HandFollow/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/PidParams.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_PidParams.lisp"
  "../msg_gen/lisp/vw.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_vw.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
