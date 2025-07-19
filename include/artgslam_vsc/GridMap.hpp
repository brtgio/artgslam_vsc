#pragma once
#include <vector>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/ViewController.hpp"

// Forward declaration para evitar dependencias circulares


class GridMap
{
private:
    int gridSize;               
    double gridResolution;      // Por ejemplo, 0.1 m por celda
    std::vector<double> posX, posY;
    std::vector<std::vector<int>> grid;

    ViewController& controller;

public:
    // Constructor configurable
    GridMap(int size, double resolution, ViewController& controller);

    // Getters
    const std::vector<double>& getRealX() const { return posX; }
    const std::vector<double>& getRealY() const { return posY; }
    int getCellIndexX(double realX) const;
    int getCellIndexY(double realY) const;

    // Manipulación de puntos
    void addPoints(double x, double y);
    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);
    void clearPoints();

    // Establecer puntos especiales
    void setStart(int i, int j);
    void setGoal(int i, int j);

    // Consultas
    int isOccupied(int i, int j) const;

    // Conversión de coordenadas reales a índice de rejilla
    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

    // Rellenar rejilla con coordenadas de celdas
    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);

    // Obtener la rejilla completa
    const std::vector<std::vector<int>>& getGrid() const;

    // Limpieza
    void clearGridMap();
    void clearSetPoints(sf::Vector2i cellIndex);

    // Dibujo del mapa con animación de AStar
    void draw(sf::RenderTarget& target, float pixelsPerMeter) const;

    // Tamaño del mapa (número de celdas)
    int getMapSize() const { return gridSize; }
};
