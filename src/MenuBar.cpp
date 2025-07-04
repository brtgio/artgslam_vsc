#include "artgslam_vsc/MenuBar.hpp"
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>



// Constructor
MenuBar::MenuBar(tgui::Gui& gui) {
    setupMenu(gui);
    setupCordLable(gui);
    setupliveToggle(gui);
}

// Separa lógica de menú para claridad
void MenuBar::setupMenu(tgui::Gui& gui) {
    menuBar = tgui::MenuBar::create();
    menuBar->setSize("100%", 20);
    menuBar->setPosition(0, "100% - 20");
    menuBar->setInvertedMenuDirection(true); // Abre hacia arriba
    gui.add(menuBar);

    menuBar->addMenu("File");
    menuBar->addMenuItem("File", "Open");
    menuBar->addMenuItem("File", "Save");
    menuBar->addMenuItem("File", "Save2Image");
    menuBar->addMenuItem("File", "Close");

    menuBar->addMenu("View");
    menuBar->addMenuItem("View", "ResetView");
    menuBar->addMenuItem("View", "ClearView");

    menuBar->addMenu("Create object");
    menuBar->addMenuItem("Create object", "WMR");

    menuBar->addMenu("Simulation");
    menuBar->addMenuItem("Simulation", "A*");


}

void MenuBar::setupCordLable(tgui::Gui &gui)
{
    coordLabel = tgui::Label::create("Grid: -, Real: -");
    coordLabel->setTextSize(14);
    coordLabel->setPosition("100% - 400", "100% - 20");
    coordLabel->getRenderer()->setBackgroundColor(tgui::Color::Transparent);
    coordLabel->getRenderer()->setTextColor(tgui::Color::Black);
    gui.add(coordLabel, "CoordLabel");
}

void MenuBar::setupliveToggle(tgui::Gui &gui)
{
liveToggle = tgui::ToggleButton::create();
    liveToggle->setSize(50, 20);
    liveToggle->setText("Live");
    liveToggle->setPosition("100% - 60", "100% - 20");

    auto renderer = liveToggle->getRenderer();
    renderer->setRoundedBorderRadius(10);
    renderer->setTextColor(tgui::Color::White);
    renderer->setBorderColor(tgui::Color::White);

    // Estado inicial (OFF)
    renderer->setBackgroundColor(tgui::Color(120, 0, 0));             // Rojo oscuro
    renderer->setBackgroundColorHover(tgui::Color(180, 60, 60));      // Hover rojo claro
    renderer->setBackgroundColorDown(tgui::Color(255, 100, 100));     // Click rojo

    gui.add(liveToggle);

    liveMode = false;

    liveToggle->onToggle([this](bool state) {
        liveMode = state;

        auto renderer = liveToggle->getRenderer();

        if (state) {
            // ON → verde brillante
            renderer->setBackgroundColor(tgui::Color(0, 180, 0));            // Verde activo
            renderer->setBackgroundColorHover(tgui::Color(80, 220, 80));     // Hover verde claro
            renderer->setBackgroundColorDown(tgui::Color(100, 255, 100));    // Click verde
        } else {
            // OFF → rojo oscuro
            renderer->setBackgroundColor(tgui::Color(120, 0, 0));
            renderer->setBackgroundColorHover(tgui::Color(180, 60, 60));
            renderer->setBackgroundColorDown(tgui::Color(255, 100, 100));
        }
    });
} // Callbacks
void MenuBar::setCallbacks(
    std::function<void()> onOpen,
    std::function<void()> onSave,
    std::function<void()> onSaveImage,
    std::function<void()> onClose,
    std::function<void()> onResetView,
    std::function<void()> onClearView,
    std::function<void()> onCreateRobot,
    std::function<void()> onSimulation)
{
    menuBar->connectMenuItem("File", "Open", [onOpen]() {
        if (onOpen) onOpen();
    });

    menuBar->connectMenuItem("File", "Save", [onSave]() {
        if (onSave) onSave();
    });

    menuBar->connectMenuItem("File", "Save2Image", [onSaveImage]() {
        if (onSaveImage) onSaveImage();
    });

    menuBar->connectMenuItem("File", "Close", [onClose]() {
        if (onClose) onClose();
    });

    menuBar->connectMenuItem("View", "ResetView", [onResetView]() {
        if (onResetView) onResetView();
    });

    menuBar->connectMenuItem("View", "ClearView", [onClearView]() {
        if (onClearView) onClearView();
    });

    menuBar->connectMenuItem("Create object", "WMR", [onCreateRobot]() {
        if (onCreateRobot) onCreateRobot();
    });

    menuBar->connectMenuItem("Simulation", "A*", [onSimulation]() {
        if (onSimulation) onSimulation();
    });
}

// Actualiza el texto del label de coordenadas
void MenuBar::updateCoordinates(const std::string& text) {
    if (coordLabel)
        coordLabel->setText(text);
}



