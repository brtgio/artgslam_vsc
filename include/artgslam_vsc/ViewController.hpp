#pragma once
#include <SFML/Graphics.hpp>

class ViewController {
public:
    ViewController(sf::RenderWindow& win, float metersPerCell, float pixelsPerMeter,sf::View& view);
 
    void handleEvent(const sf::Event& event);  // Paneo, zoom, reset
    void applyView();                          // Aplica view
    void reset();                              // Resetea al centro
    void drawGrid(sf::RenderTarget& target);   // Dibuja rejilla
    void drawAxes(sf::RenderTarget& target);   // Dibuja ejes + etiquetas
    void zoomControler(const sf::Event& event);// controla el alejamiento y acercamiento del zoom
    void view2texture(const std::string& filepath,sf::RenderWindow& window,const sf::View& view);
    sf::Vector2i windowMousePosition() const{return pixelPos;};
    float getMetersPerCell() const{return metersPerCell;};
    float getPixelsPerMeter() const{return pixelsPerMeter;};
    sf::Vector2i getMousePixelPosition() const { return mousePosition_G; }
    sf::Vector2f getMouseWorldPosition() const { return mousePosition_W; }
    sf::View getDefaultView() const {    return defaultView;};
    int getMapSizeCells() const { return mapSizeCells; }
    
    sf::View getView() const;
    float getZoom() const;
   
    sf::Vector2i getHoveredCell(int gridSize) const;
private:
    sf::RenderWindow& window;
    sf::View& view;
    sf::View defaultView;
    sf::View customDefaultView;

    bool dragging = false;
    sf::Vector2i dragStart;
    sf::Vector2f mousePosition_W;
    sf::Vector2i mousePosition_G;
    sf::Vector2i pixelPos;

    float metersPerCell;
    float pixelsPerMeter;
    const int mapSizeCells = 1000; 

    sf::Font font;
    bool fontLoaded = false;

    std::vector<std::vector<sf::Vector2f>> cellTopLeft_;  // [y][x]
    bool cellCoordsValid_ = false;
    
};


