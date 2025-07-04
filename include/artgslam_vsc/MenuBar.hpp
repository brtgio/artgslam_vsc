#pragma once
#include <TGUI/Widgets/MenuBar.hpp>
#include <TGUI/Widgets/Label.hpp>
#include <TGUI/Widgets/ToggleButton.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/TGUI.hpp>
#include <functional>

class MenuBar {
public:
    MenuBar(tgui::Gui& gui);

    void setCallbacks(
        std::function<void()> onOpen,
        std::function<void()> onSave,
        std::function<void()> onSaveImage,
        std::function<void()> onClose,
        std::function<void()> onResetView,
        std::function<void()> onClearView,
        std::function<void()> onCreateRobot,
        std::function<void()> onSimulation
    );

    void updateCoordinates(const std::string& text);
    bool getLiveMode() const { return liveMode;};
private:
    void setupMenu(tgui::Gui& gui);
    void setupCordLable(tgui::Gui& gui);
    void setupliveToggle(tgui::Gui& gui);

    tgui::MenuBar::Ptr menuBar;
    tgui::Label::Ptr coordLabel;
    tgui::ToggleButton::Ptr liveToggle;

    bool liveMode = false;
};
