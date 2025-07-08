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

    sf::View originalView = target.getView();
    target.setView(view);

    // === Dibujar ejes X y Y ===
    sf::VertexArray axes(sf::Lines);
    sf::Vector2f viewCenter = view.getCenter();
    sf::Vector2f viewSize = view.getSize();

    float left = viewCenter.x - viewSize.x / 2.f;
    float right = viewCenter.x + viewSize.x / 2.f;
    float top = viewCenter.y - viewSize.y / 2.f;
    float bottom = viewCenter.y + viewSize.y / 2.f;

    axes.append(sf::Vertex(sf::Vector2f(left, 0), sf::Color::Red));
    axes.append(sf::Vertex(sf::Vector2f(right, 0), sf::Color::Red));
    axes.append(sf::Vertex(sf::Vector2f(0, top), sf::Color::Blue));
    axes.append(sf::Vertex(sf::Vector2f(0, bottom), sf::Color::Blue));

    target.draw(axes);

    // === Etiquetas ===
    target.setView(defaultView);

    float zoomLevel = defaultView.getSize().x / view.getSize().x;
    float scaledCellSize = metersPerCell * pixelsPerMeter * zoomLevel;
    const unsigned int baseFontSize = 12;

    float textScale = std::min(zoomLevel, 1.5f);  // Limitar tamaño máximo
    if (textScale < 0.5f)
        textScale = 0.5f;  // No permitir texto demasiado pequeño

    sf::Vector2f topLeft = window.mapPixelToCoords({0, 0}, view);
    sf::Vector2f bottomRight = window.mapPixelToCoords(
        {static_cast<int>(window.getSize().x), static_cast<int>(window.getSize().y)}, view
    );

    int firstVisibleCellX = static_cast<int>(std::floor(topLeft.x / scaledCellSize));
    int firstVisibleCellY = static_cast<int>(std::floor(topLeft.y / scaledCellSize));
    int visibleCols = static_cast<int>((bottomRight.x - topLeft.x) / scaledCellSize) + 2;
    int visibleRows = static_cast<int>((bottomRight.y - topLeft.y) / scaledCellSize) + 2;

    // Eje X
    for (int i = 0; i <= visibleCols; ++i) {
        int cellX = firstVisibleCellX + i;
        if (cellX % 5 != 0) continue;

        float xpos = cellX * scaledCellSize;
        sf::Vector2i screenPos = window.mapCoordsToPixel({xpos, 0.f}, view);
        sf::Vector2f textPos(static_cast<float>(screenPos.x + 2), 4.f);

        sf::Text label(std::to_string(cellX), font, baseFontSize);
        label.setFillColor(sf::Color::Yellow);
        label.setScale(textScale, textScale);
        label.setPosition(textPos);

        sf::FloatRect textRect = label.getLocalBounds();
        sf::RectangleShape bgRect({textRect.width * textScale, textRect.height * textScale});
        bgRect.setPosition(textPos);
        bgRect.setFillColor(sf::Color(0, 0, 0, 180));

        target.draw(bgRect);
        target.draw(label);
    }

    // Eje Y
    for (int i = 0; i <= visibleRows; ++i) {
        int cellY = firstVisibleCellY + i;
        if (cellY % 5 != 0) continue;

        float ypos = cellY * scaledCellSize;
        sf::Vector2i screenPos = window.mapCoordsToPixel({0.f, ypos}, view);
        sf::Vector2f textPos(4.f, static_cast<float>(screenPos.y + 2));

        sf::Text label(std::to_string(cellY), font, baseFontSize);
        label.setFillColor(sf::Color::Yellow);
        label.setScale(textScale, textScale);
        label.setPosition(textPos);

        sf::FloatRect textRect = label.getLocalBounds();
        sf::RectangleShape bgRect({textRect.width * textScale, textRect.height * textScale});
        bgRect.setPosition(textPos);
        bgRect.setFillColor(sf::Color(0, 0, 0, 180));

        target.draw(bgRect);
        target.draw(label);
    }

    target.setView(originalView);
}






void ViewController::zoomControler(const sf::Event& event) {
    float factor = (event.mouseWheelScroll.delta > 0) ? (1.f / 1.1f) : 1.1f;
    float currentZoom = getZoom();
    float newZoom = currentZoom * factor;

    const float minZoom = 0.3f;
    const float maxZoom = 0.8f;
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
