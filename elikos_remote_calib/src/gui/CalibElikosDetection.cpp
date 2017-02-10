#include "CalibElikosDetection.h"
#include <pluginlib/class_list_macros.h>

namespace remote_calib{

CalibElikosDetection::CalibElikosDetection()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("tralala");
}

void CalibElikosDetection::initPlugin(qt_gui_cpp::PluginContext& context)
{

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
