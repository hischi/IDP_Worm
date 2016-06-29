@echo off
set COMPILER=gcc
				set COMPFLAGS=-c -fexceptions -fno-omit-frame-pointer -DMATLAB_MEX_FILE  -DMATLAB_MEX_FILE  
				set OPTIMFLAGS=-O -DNDEBUG 
				set DEBUGFLAGS=-g 
				set LINKER=gcc 
				set LINKFLAGS=-m64 -Wl,--no-undefined -shared -L"C:\Program Files\MATLAB\R2016a\extern\lib\win64\mingw64" -llibmx -llibmex -llibmat -lm -llibmwlapack -llibmwblas -Wl,"C:\Program Files\MATLAB\R2016a/extern/lib/win64/mingw64/mexFunction.def" 
				set LINKDEBUGFLAGS=-g
				set NAME_OUTPUT=-o "%OUTDIR%%MEX_NAME%%MEX_EXT%"
set PATH=C:\MATLAB\SupportPackages\R2016a\MW_MinGW_4_9\bin;C:\Program Files\MATLAB\R2016a\extern\include\win64;C:\Program Files\MATLAB\R2016a\extern\include;C:\Program Files\MATLAB\R2016a\simulink\include;C:\Program Files\MATLAB\R2016a\lib\win64;%MATLAB_BIN%;%PATH%
set INCLUDE=C:\MATLAB\SupportPackages\R2016a\MW_MinGW_4_9\include;;%INCLUDE%
set LIB=C:\MATLAB\SupportPackages\R2016a\MW_MinGW_4_9\lib;;%LIB%
set LIBPATH=C:\Program Files\MATLAB\R2016a\extern\lib\win64;%LIBPATH%

gmake SHELL="cmd" -f model_sfun.gmk
