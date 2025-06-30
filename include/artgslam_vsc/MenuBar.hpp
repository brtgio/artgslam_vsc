#pragma once
#include <TGUI/Widgets/Button.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/TGUI.hpp>

#include <functional>
class MenuBar{

    public:
    MenuBar(tgui::Gui& gui);
    void setCallbacks(
        std::function<void()> onOpen,
        std::function<void()> onSave,
        std::function<void()> onSaveImage,
        std::function<void()> onClose,
        std::function<void()> onResetView,
        std::function<void()> onClearView
    );

    void updateCoordinates(const std::string& text);

    private:
    tgui::MenuBar::Ptr menuBar;
    tgui::Label::Ptr coordLabel;

    void setupMenu();

};