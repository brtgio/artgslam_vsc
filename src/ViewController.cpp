#include "artgslam_vsc/ViewController.hpp"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>


ViewController::ViewController(sf::RenderWindow& win, float metersPerCell_, float pixelsPerMeter_, sf::View& view)
    : window(win), metersPerCell(metersPerCell_), pixelsPerMeter(pixelsPerMeter_), view(view)
{
    defaultView = window.getDefaultView();

    // Iniciamos la vista con zoom 3 (más cercana)
    view = defaultView;
    view.setCenter(0.f, 0.f);
    view.setSize(defaultView.getSize() / 3.f);
    customDefaultView = view;

    std::string package_path = ros::package::getPath("artgslam_vsc");
    std::string fontPath = package_path + "/assets/fonts/NotoSansMath-Regular.ttf";

    fontLoaded = font.loadFromFile(fontPath);
    if (!fontLoaded) {
        std::cerr << "❌ Error: No se pudo cargar la fuente en ViewController desde: " << fontPath << std::endl;
    } else {
        std::cout << "✅ Fuente cargada correctamente desde: " << fontPath << std::endl;
    }
}

void ViewController::handleEvent(const sf::Event& event) {
    if (event.type == sf::Event::MouseWheelScrolled) {
        zoomControler(event);
    }
    else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
        dragging = true;
        dragStart = sf::Mouse::getPosition(window);
    }
    else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
        dragging = false;
    }
    else if (event.type == sf::Event::MouseMoved && dragging) {
        sf::Vector2i now = sf::Mouse::getPosition(window);
        sf::Vector2f delta = window.mapPixelToCoords(dragStart) - window.mapPixelToCoords(now);
        view.move(delta);
        dragStart = now;
    }
    else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
        reset();
    }
    else if (event.type == sf::Event::MouseMoved) {
        pixelPos = sf::Mouse::getPosition(window);
        mousePosition_W = window.mapPixelToCoords(pixelPos, view);
        mousePosition_G = pixelPos;
    }
}

void ViewController::applyView() {
    window.setView(view);
}

void ViewController::reset() {
    view = customDefaultView;
}

sf::View ViewController::getView() const {
    return view;
}

float ViewController::getZoom() const {
    return view.getSize().x / defaultView.getSize().x;
}

sf::Vector2i ViewController::getHoveredCell(int gridSize) const
{
    // 1. Convertir a metros reales
    float worldX = mousePosition_W.x / pixelsPerMeter;
    float worldY = mousePosition_W.y / pixelsPerMeter;

    // 2. Compensar por el origen centrado
    float offset = (gridSize * metersPerCell) / 2.0f;

    // 3. Obtener índice de celda
    int col = static_cast<int>(std::floor((worldX + offset) / metersPerCell));
    int row = static_cast<int>(std::floor((worldY + offset) / metersPerCell));

    // 4. Validación
    if (col < 0 || col >= gridSize || row < 0 || row >= gridSize)
        return {-1, -1};

    return {col, row};
}




void ViewController::drawGrid(sf::RenderTarget& target) {
    float zoom = defaultView.getSize().x / view.getSize().x;
    float cellSize = metersPerCell * pixelsPerMeter ;
    float halfWidth = (mapSizeCells / 2) * cellSize;

    sf::VertexArray lines(sf::Lines);

    for (int i = -mapSizeCells / 2; i <= mapSizeCells / 2; ++i) {
        float pos = i * cellSize;
        lines.append(sf::Vertex({pos, -halfWidth}, sf::Color(100, 100, 100)));
        lines.append(sf::Vertex({pos, halfWidth}, sf::Color(100, 100, 100)));
        lines.append(sf::Vertex({-halfWidth, pos}, sf::Color(100, 100, 100)));
        lines.append(sf::Vertex({halfWidth, pos}, sf::Color(100, 100, 100)));
    }

    target.draw(lines);
}


void ViewController::drawAxes(sf::RenderTarget& target)
{
    if (!fontLoaded) return;

    // --------------------------------------------------------------------
    // 1) Dibujar las líneas de los ejes en coordenadas‑mundo
    // --------------------------------------------------------------------
    sf::View originalView = target.getView();
    target.setView(view);                       // ← Usa la vista actual

    sf::VertexArray axes(sf::Lines, 4);
    sf::Vector2f center = view.getCenter();
    sf::Vector2f size   = view.getSize();

    float left   = center.x - size.x * 0.5f;
    float right  = center.x + size.x * 0.5f;
    float top    = center.y - size.y * 0.5f;
    float bottom = center.y + size.y * 0.5f;

    axes[0] = sf::Vertex({left , 0.f},  sf::Color::Red);   // Eje X
    axes[1] = sf::Vertex({right, 0.f},  sf::Color::Red);
    axes[2] = sf::Vertex({0.f , top },  sf::Color::Blue);  // Eje Y
    axes[3] = sf::Vertex({0.f , bottom},sf::Color::Blue);

    target.draw(axes);

    // --------------------------------------------------------------------
    // 2) Dibujar las etiquetas en la HUD (defaultView)                    –
    //    * El tamaño de celda en mundo es fijo: metersPerCell·pixelsPerMeter
    //    * Sólo escalamos el texto en función del zoom.
    // --------------------------------------------------------------------
    target.setView(defaultView);

    const float cellSizeWorld = metersPerCell * pixelsPerMeter;   // ✅ fijo
    const float zoomLevel     = defaultView.getSize().x / view.getSize().x;

    float textScale = std::clamp(zoomLevel, 0.5f, 1.5f);          // C++17
    const unsigned baseFontSize = 12;

    // Rectángulo visible en mundo
    sf::Vector2f topLeft = window.mapPixelToCoords({0,0}, view);
    sf::Vector2f bottomRight = window.mapPixelToCoords(
        {static_cast<int>(window.getSize().x),
         static_cast<int>(window.getSize().y)}, view);

    int firstCellX = static_cast<int>(std::floor(topLeft.x / cellSizeWorld));
    int firstCellY = static_cast<int>(std::floor(topLeft.y / cellSizeWorld));
    int numCols = static_cast<int>((bottomRight.x - topLeft.x) / cellSizeWorld) + 2;
    int numRows = static_cast<int>((bottomRight.y - topLeft.y) / cellSizeWorld) + 2;

    // ---------- Etiquetas sobre el eje X ----------
    for (int i = 0; i <= numCols; ++i) {
        int cellX = firstCellX + i;
        if (cellX % 5 != 0) continue;                       // solo cada 5

        float worldX = cellX * cellSizeWorld;
        sf::Vector2i scr = window.mapCoordsToPixel({worldX, 0.f}, view);
        sf::Vector2f pos(static_cast<float>(scr.x) + 2.f, 4.f);

        sf::Text txt(std::to_string(cellX), font, baseFontSize);
        txt.setFillColor(sf::Color::Yellow);
        txt.setScale(textScale, textScale);
        txt.setPosition(pos);

        sf::FloatRect b = txt.getLocalBounds();
        sf::RectangleShape bg({b.width * textScale, b.height * textScale});
        bg.setPosition(pos);
        bg.setFillColor(sf::Color(0,0,0,180));

        target.draw(bg);
        target.draw(txt);
    }

    // ---------- Etiquetas sobre el eje Y ----------
    for (int i = 0; i <= numRows; ++i) {
        int cellY = firstCellY + i;
        if (cellY % 5 != 0) continue;

        float worldY = cellY * cellSizeWorld;
        sf::Vector2i scr = window.mapCoordsToPixel({0.f, worldY}, view);
        sf::Vector2f pos(4.f, static_cast<float>(scr.y) + 2.f);

        sf::Text txt(std::to_string(cellY), font, baseFontSize);
        txt.setFillColor(sf::Color::Yellow);
        txt.setScale(textScale, textScale);
        txt.setPosition(pos);

        sf::FloatRect b = txt.getLocalBounds();
        sf::RectangleShape bg({b.width * textScale, b.height * textScale});
        bg.setPosition(pos);
        bg.setFillColor(sf::Color(0,0,0,180));

        target.draw(bg);
        target.draw(txt);
    }

    // --------------------------------------------------------------------
    // 3) Restaurar la vista original
    // --------------------------------------------------------------------
    target.setView(originalView);
}








void ViewController::zoomControler(const sf::Event& event) {
    float factor = (event.mouseWheelScroll.delta > 0) ? (1.f / 1.1f) : 1.1f;
    float currentZoom = getZoom();
    float newZoom = currentZoom * factor;

    const float minZoom = 0.1f;
    const float maxZoom = 1.0f;
    std::cout << "newZoom" << newZoom << std::endl;

    if (newZoom < minZoom || newZoom > maxZoom) {
        std::cout << "Zoom fuera de rango [" << minZoom << ", " << maxZoom << "], operación cancelada\n";
        return;
    }

    view.zoom(factor);
}

void ViewController::view2texture(const std::string& filepath, sf::RenderWindow& window, const sf::View& view) {
    sf::Vector2f viewZise = view.getSize();
    sf::RenderTexture renderTexture;

    renderTexture.create(static_cast<unsigned int>(viewZise.x), static_cast<unsigned int>(viewZise.y));
    renderTexture.setView(view);
    renderTexture.clear(sf::Color::Transparent);
}
