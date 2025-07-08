#include "artgslam_vsc/GridMap.hpp"


GridMap::GridMap(int size, double resolution,ViewController& controller)
    : gridSize(size), gridResolution(resolution),controller(controller)
{
    // Inicializamos el grid vacío de una vez
    grid.assign(gridSize, std::vector<int>(gridSize, 0));
}

int GridMap::getCellIndexX(double realX) const
{
    double offset = (gridSize * gridResolution) / 2.0;  // si tu origen está al centro
    int j = static_cast<int>(std::floor((realX + offset) / gridResolution));
    if (j < 0) j = 0;
    if (j >= gridSize) j = gridSize - 1;
    return j;
}

int GridMap::getCellIndexY(double realY) const
{
    double offset = (gridSize * gridResolution) / 2.0;  // igual al eje Y
    int i = static_cast<int>(std::floor((realY + offset) / gridResolution));
    if (i < 0) i = 0;
    if (i >= gridSize) i = gridSize - 1;
    return i;
}

void GridMap::addPoints(double x, double y)
{
    posX.push_back(x);
    posY.push_back(y);
}

void GridMap::setPoints(const std::vector<double> &newX, const std::vector<double> &newY)
{
    posX = newX;
    posY = newY;
}

void GridMap::clearPoints()
{
    posX.clear();
    posY.clear();
}

void GridMap::xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                      std::vector<int>& xGrid, std::vector<int>& yGrid)
{
const double resolution = 0.1;
    const int gridSize = 1000; // Mapa de 1000x1000 celdas
    const int halfGrid = gridSize / 2;

    size_t n = std::min(x.size(), y.size());
    xGrid.resize(n);
    yGrid.resize(n);

    for (size_t i = 0; i < n; ++i) {
        int gx = static_cast<int>(std::round(x[i] / resolution));
        int gy = static_cast<int>(std::round(y[i] / resolution));

        // Centrar el origen en el medio del grid
        xGrid[i] = gx + halfGrid;
        yGrid[i] = gy + halfGrid;
    }
}

void GridMap::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
        const int gridSize = 1000;

    grid.assign(gridSize, std::vector<int>(gridSize, 0));

    for (size_t i = 0; i < xGrid.size(); ++i) {
        int x = xGrid[i];
        int y = yGrid[i];

        if (x >= 0 && x < gridSize && y >= 0 && y < gridSize) {
            grid[y][x] = 1;
        }
    }
}

const std::vector<std::vector<int>>& GridMap::getGrid() const
{
    return grid;
}

void GridMap::clearGridMap()
{
    grid.assign(gridSize, std::vector<int>(gridSize, 0));
}

void GridMap::draw(sf::RenderTarget& target, float pixelsPerMeter) const
{
    if (grid.empty() || grid[0].empty()) return;

    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    float cellSize = gridResolution * pixelsPerMeter;

    float zoom = controller.getZoom();  // Se asume que `controller` es un puntero válido a ViewController

    // Offset centrado, escalado con zoom
    float offsetX = - (static_cast<float>(cols) / 2.f) * (cellSize / zoom);
    float offsetY = - (static_cast<float>(rows) / 2.f) * (cellSize / zoom);

    sf::RectangleShape cellShape;
    cellShape.setFillColor(sf::Color::Yellow);
    cellShape.setSize(sf::Vector2f(cellSize / zoom, cellSize / zoom)); // Ajustar tamaño al zoom

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (grid[row][col] == 1) {
                float x = offsetX + col * (cellSize / zoom);
                float y = offsetY + row * (cellSize / zoom);

                cellShape.setPosition(x, y);
                target.draw(cellShape);
            }
        }
    }
}


