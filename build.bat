@echo off

IF NOT EXIST build\win mkdir build\win

set CommonFlags=-Oi -nologo /wd4312 /wd4577 /wd4530 /O2

set Libs=..\..\libs\windows\glew\lib\Release\x64\glew32s.lib opengl32.lib ..\..\libs\windows\assimp\lib64\assimp.lib
set Includes=/I ..\..\libs\glm /I ..\..\libs\windows\glew\include /I ..\..\libs\assimp\include /I ..\..\libs\stb /I ..\..\libs\base /I ..\..\libs\windows\dirent

set EngineLibs=/I ..\..\libs\windows\SDL2\include ..\..\libs\windows\SDL2\lib\x64\SDL2main.lib ..\..\libs\windows\SDL2\lib\x64\SDL2.lib

pushd build\win
cl ..\..\src\main.cpp %CommonFlags% %Libs% %Includes% %EngineLibs% /Fe:pathtracer.exe
popd
