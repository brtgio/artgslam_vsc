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
    view.setSize(defaultView.getSize() / 3.f);  // tamaño más pequeño → todo se ve más grande
    customDefaultView = view;

    // Obtener ruta del paquete y cargar fuente
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
        sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
        //std::cout << "Mouse moved. Pixel: (" << pixelPos.x << ", " << pixelPos.y << ")\n";
        sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos, view);

        mousePosition_G = pixelPos;
        mousePosition_W = worldPos;
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
    // Zoom: cuánto está acercada la vista respecto a la vista por defecto
        return view.getSize().x / defaultView.getSize().x;
}

void ViewController::drawGrid(sf::RenderTarget& target) {
    float zoom = defaultView.getSize().x / view.getSize().x;  // inverso de getZoom()
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

    // === Dibujar Ejes ===
    target.setView(view);

    sf::VertexArray axes(sf::Lines);

    sf::Vector2f viewCenter = view.getCenter();
    sf::Vector2f viewSize = view.getSize();

    float left = viewCenter.x - viewSize.x / 2.f;
    float right = viewCenter.x + viewSize.x / 2.f;
    float top = viewCenter.y - viewSize.y / 2.f;
    float bottom = viewCenter.y + viewSize.y / 2.f;

    // Eje X (rojo)
    axes.append(sf::Vertex(sf::Vector2f(left, 0), sf::Color::Red));
    axes.append(sf::Vertex(sf::Vector2f(right, 0), sf::Color::Red));

    // Eje Y (verde)
    axes.append(sf::Vertex(sf::Vector2f(0, top), sf::Color::Green));
    axes.append(sf::Vertex(sf::Vector2f(0, bottom), sf::Color::Green));

    target.draw(axes);

    // === Dibujar Etiquetas ===
    target.setView(defaultView);

    sf::Text label;
    label.setFont(font);
    label.setCharacterSize(16);
    label.setFillColor(sf::Color::Yellow);

    // Obtener límites visibles en coordenadas del mundo
    sf::Vector2f topLeft = window.mapPixelToCoords({0, 0}, view);
    sf::Vector2f bottomRight = window.mapPixelToCoords(
        sf::Vector2i(window.getSize().x, window.getSize().y), view);

    // Calcular los índices mínimos y máximos visibles
    int minX = static_cast<int>(std::floor(topLeft.x / (metersPerCell)));
    int maxX = static_cast<int>(std::ceil(bottomRight.x / (metersPerCell)));

    int minY = static_cast<int>(std::floor(topLeft.y / (metersPerCell)));
    int maxY = static_cast<int>(std::ceil(bottomRight.y / (metersPerCell)));

    // Decidir cada cuántas líneas numerar (para no saturar)
    int skip = 1;
    if (cellSize < 50) skip = 2;
    if (cellSize < 25) skip = 5;
    if (cellSize < 12) skip = 10;

    // === Etiquetas Eje X ===
    for (int i = minX; i <= maxX; ++i) {
        if (i % skip != 0) continue;

        sf::Vector2f worldPos(i * metersPerCell, 0.f);
        sf::Vector2i screenPos = window.mapCoordsToPixel(worldPos, view);

        label.setString(std::to_string(i));
        sf::FloatRect bounds = label.getLocalBounds();

        label.setOrigin(bounds.width / 2.f, 0.f);
        label.setPosition(static_cast<float>(screenPos.x),
                           static_cast<float>(window.getSize().y) - bounds.height - 4.f);

        if (label.getPosition().x < 0 || label.getPosition().x > window.getSize().x) continue;

        target.draw(label);
    }

    // === Etiquetas Eje Y ===
    for (int i = minY; i <= maxY; ++i) {
        if (i % skip != 0) continue;

        sf::Vector2f worldPos(0.f, i * metersPerCell);
        sf::Vector2i screenPos = window.mapCoordsToPixel(worldPos, view);

        label.setString(std::to_string(i));
        sf::FloatRect bounds = label.getLocalBounds();

        label.setOrigin(0.f, bounds.height / 2.f);
        label.setPosition(4.f, static_cast<float>(screenPos.y));

        if (label.getPosition().y < 0 || label.getPosition().y > window.getSize().y) continue;

        target.draw(label);
    }

    target.setView(originalView);
}










void ViewController::zoomControler(const sf::Event& event) {
    float factor = (event.mouseWheelScroll.delta > 0) ? (1.f / 1.1f) : 1.1f;

    float currentZoom = getZoom();
    float newZoom = currentZoom * factor;

    const float minZoom = 0.05f;  // vista más pequeña (zoom más cerca)
    const float maxZoom = .50f;  // vista más grande (zoom más lejos)
            std::cout<<"newZoom"<< newZoom<<std::endl;

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

