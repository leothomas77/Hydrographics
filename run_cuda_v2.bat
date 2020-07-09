cd bin\win64
rem START /w /high NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testFilm -selectedModel=0 -dippingVelocity=-0.3

rem NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testMesh -selectedModel=1 -dippingVelocity=-0.3
rem NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testMesh -selectedModel=2 -dippingVelocity=-0.3
rem NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testMesh -selectedModel=0 -dippingVelocity=-0.3


rem sem start log no arquivo
NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testVoxel -selectedModel=0 -dippingVelocity=-0.3 >> testVoxelModel0.log
NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testVoxel -selectedModel=1 -dippingVelocity=-0.3 >> testVoxelModel1.log
NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testVoxel -selectedModel=2 -dippingVelocity=-0.3 >> testVoxelModel2.log

NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testFilm -selectedModel=0 -dippingVelocity=-0.3 >> testFilmModel0.log
NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testFilm -selectedModel=1 -dippingVelocity=-0.3 >> testFilmModel1.log
NvFlexDemoReleaseCUDA_x64.exe -device=0 -vsync=0 -testFilm -selectedModel=2 -dippingVelocity=-0.3 >> testFilmModel2.log

cd ../..
