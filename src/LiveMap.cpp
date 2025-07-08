#include <artgslam_vsc/LiveMap.hpp>
#include <iostream> // para debug

LiveMap::LiveMap(int size, double resolution,ViewController& controller)
    : gridSize(size), gridResolution(resolution),controller(controller)
{
    // Inicializa el grid en ceros (0 = libre)
    grid.resize(gridSize, std::vector<int>(gridSize, 0));

    // Limpia los vectores de puntos
    posX.clear();
    posY.clear();
}

void LiveMap::addPoint(double x, double y){
      if (!originSet) {
        originX = x;
        originY = y;
        originSet = true;
    }
    posX.push_back(x);
    posY.push_back(y);
}

void LiveMap::setPoints(const std::vector<double>& newX, const std::vector<double>& newY){
    posX = newX;
    posY = newY;
}

void LiveMap::clearPoints(){
    posX.clear();
    posY.clear();
    originSet = false;
}

void LiveMap::clearGrid()
{
    for (auto& row : grid) {
        std::fill(row.begin(), row.end(), 0);
    }
}

void LiveMap::xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                      std::vector<int>& xGrid, std::vector<int>& yGrid)
{
       if (x.size() != y.size() || !originSet) return;

    xGrid.clear();
    yGrid.clear();

    int halfGrid = gridSize / 2;

    for (size_t i = 0; i < x.size(); ++i) {
        double shiftedX = x[i] - originX;
        double shiftedY = y[i] - originY;

        int xIdx = static_cast<int>(std::round(shiftedX / gridResolution)) + halfGrid;
        int yIdx = static_cast<int>(std::round(shiftedY / gridResolution)) + halfGrid;

        if (xIdx < 0 || xIdx >= gridSize || yIdx < 0 || yIdx >= gridSize)
            continue;

        xGrid.push_back(xIdx);
        yGrid.push_back(yIdx);
    }
}


void LiveMap::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
    if (xGrid.size() != yGrid.size()) return;

    clearGrid(); // Limpiar antes de llenar

    for (size_t i = 0; i < xGrid.size(); ++i) {
        int xIdx = xGrid[i];
        int yIdx = yGrid[i];

        grid[yIdx][xIdx] = 1; // Marca la celda como ocupada
    }
}

void LiveMap::updateGridFromPoints()
{
    if (posX.empty() || posY.empty()) return;

    std::vector<int> xGrid, yGrid;
    xy2Grid(posX, posY, xGrid, yGrid);
    fillGrid(xGrid, yGrid);
}

void LiveMap::drawLiveMap(sf::RenderTarget& target) const
{
    if (grid.empty()) return;

    float metersPerCell = controller.getMetersPerCell();   // 0.1
    float pixelsPerMeter = controller.getPixelsPerMeter(); // 50.0

    float cellSize = metersPerCell * pixelsPerMeter;       // 5 píxeles por celda

    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    int halfCols = cols / 2;
    int halfRows = rows / 2;

    float zoom = controller.getZoom();

    sf::RectangleShape cellShape;
    cellShape.setFillColor(sf::Color::Magenta);
    cellShape.setSize(sf::Vector2f(cellSize / zoom, cellSize / zoom));  // tamaño ajustado

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (grid[row][col] == 1) {
                int cellX = col - halfCols;
                int cellY = row - halfRows;

                float x = static_cast<float>(cellX) * cellSize/zoom;
                float y = static_cast<float>(cellY) * cellSize/zoom;

                cellShape.setPosition(x, y);
                target.draw(cellShape);
            }
        }
    }
}


