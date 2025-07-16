#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include "artgslam_vsc/MapViewer.hpp"


int main(int argc, char** argv)
{
    // Inicializar ROS
    ros::init(argc, argv, "artgslam_vsc_node");

    // Crear ventana SFML
    sf::RenderWindow window(sf::VideoMode(1024, 768), "ARTG SLAM Visualizer");
    window.setFramerateLimit(60);  // Limitar FPS para estabilidad

    // Crear objeto GridMap
    

    // Crear MapViewer (con ROS manejado internamente si RosHandler es parte de MapViewer)
    MapViewer mapViewer(window);

    // Bucle principal
    while (ros::ok() && mapViewer.isRunning())
    {
        // Procesar eventos de la ventana (mouse, teclado, etc.)
        mapViewer.processEvent();

        // Actualizar lógica interna

            mapViewer.update();

        

        // Renderizar contenido gráfico
        mapViewer.render();

        // Procesar callbacks de ROS (joy, sonar, etc.)
        ros::spinOnce();
    }

    return 0;
}
