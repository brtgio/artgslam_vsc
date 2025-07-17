#pragma once
#include <vector>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/ViewController.hpp"


class GridMap
{
private:
    int gridSize;               
    double gridResolution;      // e.g., 0.1 m por celda
    std::vector<double> posX, posY;
    std::vector<std::vector<int>> grid;

    ViewController& controller;

public:
    GridMap(int size , double resolution ,ViewController& controller);  // Constructor configurable

    const std::vector<double>& getRealX() const{ return posX;}
    const std::vector<double>& getRealY() const{ return posY;}
    int getCellIndexX(double realX) const;
    int getCellIndexY(double realY) const;

    void addPoints(double x, double y);
    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);
    void clearPoints();

    void setStart(int i,int j);
    void setGoal(int i,int j);
    bool isOccupied(int i,int j) const{ return grid[j][i];}
    

    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);
    const std::vector<std::vector<int>>& getGrid() const;

    void clearGridMap();
    void clearSetPoints(sf::Vector2i cellIndex);
    void draw(sf::RenderTarget& target, float pixelsPerMeter) const;
    int getMapSize() const { return gridSize; }
};

