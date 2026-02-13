g++ -std=c++17 -g Route.cpp -o Route
rm *.sam
./Route Test.rect li
python3 Visualize_Tiles.py