#include <boost/circular_buffer.hpp>
#include <memory>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_rosout.hpp"

namespace gui_node
{

WidgetRosout::WidgetRosout(std::shared_ptr<GuiNode> gui_node_ptr, const std::string &ros_data_name,
                           const std::string &window_name, int max_table_size)
    : Widget(gui_node_ptr), ros_data_name(ros_data_name), window_name(window_name),
      messages(boost::circular_buffer<rosout_message>(max_table_size))
{
}

WidgetRosout::WidgetRosout(std::shared_ptr<GuiNode> gui_node_ptr, const std::string &ros_data_name,
                           const std::string &window_name)
    : Widget(gui_node_ptr), ros_data_name(ros_data_name), window_name(window_name),
      messages(boost::circular_buffer<rosout_message>(1000))
{
}

bool WidgetRosout::isUniqueMessage(rcl_interfaces::msg::Log::SharedPtr msg)
{
    if (messages.size() == 0)
    {
        return true;
    }
    rosout_message first_message = messages.front();
    if (first_message.level != msg->level || first_message.name != msg->name || first_message.msg != msg->msg ||
        first_message.file != msg->file || first_message.function != msg->function || first_message.line != msg->line)
    {
        return true;
    }
    return false;
}

void WidgetRosout::draw()
{
    using RosoutSubscriber = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;
    std::shared_ptr<RosoutSubscriber> subscriber = this->gui_node->getRosData(ros_data_name)->as<RosoutSubscriber>();
    rcl_interfaces::msg::Log::SharedPtr message = subscriber->getData();
    if (message != nullptr && isUniqueMessage(message))
    {
        messages.push_front(
            {message->level, message->name, message->msg, message->file, message->function, message->line});
    }
    ImGui::Begin(window_name.c_str());
    ImGui::SetWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
    if (ImGui::BeginTable("rosout_table", 6, flags, ImVec2(0.0f, 0.0f)))
    {
        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_NoHide);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Message");
        ImGui::TableSetupColumn("File");
        ImGui::TableSetupColumn("Function");
        ImGui::TableSetupColumn("Line");
        ImGui::TableHeadersRow();
        for (rosout_message message : messages)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s", rosout_level_map[message.level].c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message.name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message.msg.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message.file.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message.function.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%d", message.line);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace gui_node
