FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "cfg/cpp/semantic_label_3d/labelviewerConfig.h"
  "docs/labelviewerConfig.dox"
  "docs/labelviewerConfig-usage.dox"
  "src/semantic_label_3d/cfg/labelviewerConfig.py"
  "docs/labelviewerConfig.wikidoc"
  "cfg/cpp/semantic_label_3d/pcmergerConfig.h"
  "docs/pcmergerConfig.dox"
  "docs/pcmergerConfig-usage.dox"
  "src/semantic_label_3d/cfg/pcmergerConfig.py"
  "docs/pcmergerConfig.wikidoc"
  "cfg/cpp/semantic_label_3d/labelingConfig.h"
  "docs/labelingConfig.dox"
  "docs/labelingConfig-usage.dox"
  "src/semantic_label_3d/cfg/labelingConfig.py"
  "docs/labelingConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
