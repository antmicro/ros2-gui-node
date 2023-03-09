#ifndef GUI_NODE_WIDGET_WIDGET_HPP
#define GUI_NODE_WIDGET_WIDGET_HPP

#include "gui_node/gui_engine.hpp"
#include "gui_node/gui_node.hpp"
#include <functional>
#include <memory>
#include <string>

namespace gui_node
{

class GuiNode; ///< Forward declaration

/**
 * Base class for all GUI widgets.
 */
class Widget
{
protected:
    std::shared_ptr<GuiNode> gui_node; ///< The GUI node that this widget is attached to

public:
    /**
     * Constructor.
     *
     * @param gui_node The GUI node that this widget is attached to
     */
    Widget(std::shared_ptr<GuiNode> gui_node) : gui_node(gui_node) {}

    /**
     * Destructor.
     */
    virtual ~Widget() = 0;

    /**
     * Virtual function that is called to draw the widget using the ImGui API.
     */
    virtual void draw() = 0;
};

} // namespace gui_node
#endif // GUI_NODE_WIDGET_WIDGET_HPP
