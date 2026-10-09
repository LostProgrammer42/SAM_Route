g++ -std=c++17 -g SAM_Route.cpp -o Route
rm SAM_Files/*.sam
./Route Test_Cells/g0n1n2naa_01ox0.rect polysilicon,m1
python3 Visualize_Tiles.py