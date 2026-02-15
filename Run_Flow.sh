g++ -std=c++17 -g Route.cpp -o Route
rm *.sam
./Route INVX8.rect polysilicon,m1
python3 Visualize_Tiles.py