#include "artgslam_vsc/RightClickMapMenu.hpp"



#include "artgslam_vsc/RightClickMapMenu.hpp"

RightClickMapMenu::RightClickMapMenu(tgui::Gui& guiRef)
    : gui(guiRef)
{
    std::string package_path = ros::package::getPath("artgslam_vsc");
    std::string formPath = package_path + "/assets/forms/Right_Click_Menu.txt";

    try {
        // Crear contenedor para encapsular los widgets del form
        container = tgui::Group::create();
        container->loadWidgetsFromFile(formPath);

        // Agregar el contenedor a la GUI principal (sin borrar la MenuBar)
        gui.add(container);

        // Obtener el panel y botones desde el contenedor, no directamente desde gui
        panel = container->get<tgui::ChildWindow>("ChildWindow1");
        startBtn = panel->get<tgui::Button>("start");
        goalBtn = panel->get<tgui::Button>("goal");
        clearBtn = panel->get<tgui::Button>("Clear");
        clearViewBtn = panel->get<tgui::Button>("ClearView");

        if (panel) {
            panel->setVisible(false);
        }

    } catch (const tgui::Exception& e) {
        std::cerr << "❌ Error loading GUI form: " << e.what() << std::endl;
    }
}


void RightClickMapMenu::setupWiggets()
{
}
void RightClickMapMenu::show(float x, float y) {
    if (panel) {
        panel->setPosition(x, y);
        panel->setVisible(true);
        visible = true;
    }
}

void RightClickMapMenu::hide() {
    if (panel) {
        panel->setVisible(false);
        visible = false;
    }
}

void RightClickMapMenu::setVisible(bool show){
    if (panel) {
        panel->setVisible(show);
        visible = show;
    }
}

void RightClickMapMenu::connectSignals() {
    // TODO: conectar señales si panel y botones fueron cargados correctamente

    if (startBtn)
        startBtn->onPress([](){ std::cout << "Start pressed\n"; });
    if (goalBtn)
        goalBtn->onPress([](){ std::cout << "Goal pressed\n"; });
    if (clearBtn)
        clearBtn->onPress([](){ std::cout << "Clear selection pressed\n"; });
    if (clearViewBtn)
        clearViewBtn->onPress([](){ std::cout << "Clear View pressed\n"; });
}
