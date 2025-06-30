#include <SFML/Graphics.hpp>
#include "MapViewer.hpp"
#include "GridMap.hpp"  // Incluye si MapViewer.hpp no lo incluye

int main()
{
    // Crear ventana SFML
    sf::RenderWindow window(sf::VideoMode(800, 600), "Test MapViewer");
    
    // Crear objeto GridMap
    GridMap grid;           
    
    // Crear objeto MapViewer, pasando la referencia al mismo GridMap
    MapViewer mapViewer(window, grid);

    // Loop principal
    while (mapViewer.isRunning())
    {
        mapViewer.processEvent();  // Procesar eventos
        mapViewer.update();        // Actualizaciones lógicas
        mapViewer.render();        // Renderizar todo
    }

    return 0;
}
