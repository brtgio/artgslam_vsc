#include "artgslam_vsc/MenuBar.hpp"
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

MenuBar::MenuBar(tgui::Gui& gui)
{
    menuBar = tgui::MenuBar::create();
    menuBar->setSize("100%", 20);
    menuBar->setPosition(0, "100% - 20");
    menuBar->setInvertedMenuDirection(true); // Abre hacia arriba
    gui.add(menuBar);

    // Label de coordenadas
    coordLabel = tgui::Label::create("Grid: -, Real: -");
    coordLabel->setTextSize(14);
    coordLabel->setPosition("100% - 320", "100% - 20");
    coordLabel->getRenderer()->setBackgroundColor(tgui::Color::Transparent);
    coordLabel->getRenderer()->setTextColor(tgui::Color::Black);
    gui.add(coordLabel, "CoordLabel");

    setupMenu();
}

void MenuBar::setupMenu()
{
    menuBar->addMenu("File");
    menuBar->addMenuItem("File", "Open");
    menuBar->addMenuItem("File", "Save");
    menuBar->addMenuItem("File", "Save2Image");
    menuBar->addMenuItem("File", "Close");

    menuBar->addMenu("View");
    menuBar->addMenuItem("View", "ResetView");
    menuBar->addMenuItem("View", "ClearView");
}

void MenuBar::setCallbacks(
    std::function<void()> onOpen,
    std::function<void()> onSave,
    std::function<void()> onSaveImage,
    std::function<void()> onClose,
    std::function<void()> onResetView,
    std::function<void()> onClearView)
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
}

void MenuBar::updateCoordinates(const std::string& text)
{
    if (coordLabel) {
        coordLabel->setText(text);
    }
}
