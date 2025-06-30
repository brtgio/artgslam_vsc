#include "ViewController.hpp"
#include <cmath>
#include <iostream>

ViewController::ViewController(sf::RenderWindow& win, float metersPerCell_, float pixelsPerMeter_,sf::View& view)
    : window(win), metersPerCell(metersPerCell_), pixelsPerMeter(pixelsPerMeter_) ,view(view)
{
   defaultView = window.getDefaultView();

    
   
    view = defaultView;
    view.setCenter(0.f, 0.f);
    view.setSize(defaultView.getSize());
    customDefaultView=view;

    fontLoaded = font.loadFromFile("NotoSansMath-Regular.ttf");
    if (!fontLoaded){
         std::cerr << "No se pudo cargar la fuente para ViewController\n";
        
    }
       
}

void ViewController::handleEvent(const sf::Event& event) {
    if (event.type == sf::Event::MouseWheelScrolled) {
        zoomControler(event);
        
    } else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
        dragging = true;
        dragStart = sf::Mouse::getPosition(window);
    } else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
        dragging = false;
    } else if (event.type == sf::Event::MouseMoved && dragging) {
        sf::Vector2i now = sf::Mouse::getPosition(window);
        sf::Vector2f delta = window.mapPixelToCoords(dragStart) - window.mapPixelToCoords(now);
        view.move(delta);
        dragStart = now;
    } else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
        reset();
    } else if(event.type == sf::Event::MouseMoved){
        sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
        std::cout << "Mouse moved. Pixel: (" << pixelPos.x << ", " << pixelPos.y << ")\n";
        sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos, view);

        // Guarda para consulta externa
        mousePosition_G = pixelPos;
        mousePosition_W = worldPos;
    }

}

void ViewController::applyView() {
    window.setView(view);
}

void ViewController::reset() {
    view=customDefaultView;
}

sf::View ViewController::getView() const {
    return view;
}

float ViewController::getZoom() const
{
        return defaultView.getSize().x / view.getSize().x;
}

void ViewController::drawGrid(sf::RenderTarget& target) {
    float zoom = defaultView.getSize().x / view.getSize().x;
    float cellSize = metersPerCell * pixelsPerMeter * zoom;
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
void ViewController::drawAxes(sf::RenderTarget& target) {
    if (!fontLoaded) return;

    float zoom = defaultView.getSize().x / view.getSize().x;
    float cellSize = metersPerCell * pixelsPerMeter * zoom;

    sf::View originalView = target.getView();

    // Dibujar ejes en coordenadas del mundo (usando la vista activa)
    target.setView(view);

    float halfWidth = (mapSizeCells / 2) * cellSize;
    sf::VertexArray axes(sf::Lines);
    axes.append(sf::Vertex({0.f, -halfWidth}, sf::Color::Red));
    axes.append(sf::Vertex({0.f, halfWidth}, sf::Color::Red));
    axes.append(sf::Vertex({-halfWidth, 0.f}, sf::Color::Green));
    axes.append(sf::Vertex({halfWidth, 0.f}, sf::Color::Green));
    target.draw(axes);

    // Ahora dibujar etiquetas fijas en pantalla pero representando la celda correspondiente
    target.setView(defaultView);  // Vista fija en pantalla

    sf::Text label;
    label.setFont(font);
    label.setFillColor(sf::Color::Yellow);
    label.setCharacterSize(14);

    // Etiquetas en eje X: parte superior de la pantalla
    int step = static_cast<int>(cellSize * 5);
    if (step < 1) step = 1;

    for (int screenX = 0; screenX < static_cast<int>(window.getSize().x); screenX += step) {
        // Convertimos desde pantalla (screenX, 0) a coordenada del mundo
        sf::Vector2f worldCoords = window.mapPixelToCoords({screenX, 0}, view);
        int cellIndex = static_cast<int>(std::round(worldCoords.x / cellSize));

        label.setString(std::to_string(cellIndex));
        sf::FloatRect bounds = label.getLocalBounds();
        label.setOrigin(bounds.width / 2.f, 0.f);
        label.setPosition(static_cast<float>(screenX), 2.f);
        target.draw(label);
    }

    // Etiquetas en eje Y: parte izquierda de la pantalla
    for (int screenY = 0; screenY < static_cast<int>(window.getSize().y); screenY += step) {
        sf::Vector2f worldCoords = window.mapPixelToCoords({0, screenY}, view);
        int cellIndex = static_cast<int>(std::round(worldCoords.y / cellSize));

        label.setString(std::to_string(cellIndex));
        sf::FloatRect bounds = label.getLocalBounds();
        label.setOrigin(bounds.width + 2.f, bounds.height / 2.f);
        label.setPosition(14.f, static_cast<float>(screenY));
        target.draw(label);
    }

    target.setView(originalView);
}


void ViewController::zoomControler(const sf::Event& event)
{
    float factor = (event.mouseWheelScroll.delta > 0) ? 1.f / 1.1f : 1.1f;

    float currentZoom = getZoom();
    float newZoom = currentZoom * (1.f / factor);  // ajustar porque el tamaño cambia inversamente

    const float minZoom = 0.5f;  // zoom más alejado → vista 2×
    const float maxZoom = 2.0f;  // zoom más cercano → vista 0.5×

    if (newZoom < minZoom || newZoom > maxZoom) {
        std::cout << "Zoom fuera de rango [" << minZoom << ", " << maxZoom << "], operación cancelada\n";
        return;
    }

    view.zoom(factor);
}







void ViewController::view2texture(const std::string &filepath, sf::RenderWindow &window, const sf::View &view)
{
    sf::Vector2f viewZise=view.getSize();
    sf::RenderTexture renderTexture;

    renderTexture.create(static_cast<unsigned int>(viewZise.x),static_cast<unsigned int>(viewZise.y));
    renderTexture.setView(view);
    renderTexture.clear(sf::Color::Transparent);

}

