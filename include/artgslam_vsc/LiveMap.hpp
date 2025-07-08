#pragma once
#include <vector>
#include <math.h>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/ViewController.hpp"
class LiveMap {
public:
    LiveMap(int size , double resolution ,ViewController& controller);

    void addPoint(double x, double y);
    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);
    void clearPoints();
    void clearGrid();
    
    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);
    const std::vector<std::vector<int>>& getGrid() const{return grid;};
    void updateGridFromPoints();

    void drawLiveMap(sf::RenderTarget& target) const;

private:
    ViewController& controller;
    int gridSize;
    double gridResolution;

    std::vector<double> posX, posY;
    std::vector<std::vector<int>> grid;

    bool originSet = false;
    double originX = 0.0, originY = 0.0;

   
};



