#!/bin/bash

set -e

app_name='Pathtracer.app'
executable_name='explore'

shared_flags='
-g -Wall -Wextra -std=c++11 -Wno-missing-field-initializers -Wno-unused-parameter
'
optimalization='-O0'
internal=''

libraries="
  -I./libs/assimp/include
  ./build/$app_name/Contents/Frameworks/libassimp.3.1.1.dylib

  -I./libs/stb
  -I./libs/glm

  -F./build/$app_name/Contents/Frameworks
  -rpath @executable_path/../Frameworks
"

engine_main="src/main.cpp $libraries"
engine_flags="
  -framework SDL2
"

build_engine() {
  echo 'Building engine'
  echo '=================='

  clang++ $engine_main -o build/$app_name/Contents/MacOS/$executable_name $shared_flags $engine_flags $optimalization $internal
}

main() {
  release=false
  libs=false

  for i in "$@"
  do
    case $i in
      -i|--internal)
        internal='-DINTERNAL'
      ;;
      -r|--release)
        optimalization='-O3'
      ;;
    esac
    shift
  done

  build_engine
}

main $@

