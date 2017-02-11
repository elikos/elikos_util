#include <elikos_remote_calib/CalibElikosDetection.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QPushButton>

namespace remote_calib{

CalibElikosDetection::CalibElikosDetection()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("tralala");
}

CalibElikosDetection::~CalibElikosDetection()
{
    //Normally rqt deletes the widget_
}

void CalibElikosDetection::initPlugin(qt_gui_cpp::PluginContext& context)
{
    QStringList argv = context.argv();
    widget_ = new QWidget();

    ui_.setupUi(widget_);
    context.addWidget(widget_);
}

void CalibElikosDetection::shutdownPlugin()
{

}

void CalibElikosDetection::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings)
{

} 

void CalibElikosDetection::restoreSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings)
{

}

}

PLUGINLIB_DECLARE_CLASS(remote_calib, CalibElikosDetection, remote_calib::CalibElikosDetection, rqt_gui_cpp::Plugin)
