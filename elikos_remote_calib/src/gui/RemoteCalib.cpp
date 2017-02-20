#include <elikos_remote_calib/RemoteCalib.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QPushButton>

namespace remote_calib{

RemoteCalib::RemoteCalib()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("tralala");
}

RemoteCalib::~RemoteCalib()
{
}

void RemoteCalib::initPlugin(qt_gui_cpp::PluginContext& context)
{
    QStringList argv = context.argv();
    widget_ = new CalibWidget();

    //ui_.setupUi(widget_);
    //Normally qt deletes the widget_
    context.addWidget(widget_);
}

void RemoteCalib::shutdownPlugin()
{

}

void RemoteCalib::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings)
{

} 

void RemoteCalib::restoreSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings)
{

}

}

PLUGINLIB_DECLARE_CLASS(remote_calib, RemoteCalib, remote_calib::RemoteCalib, rqt_gui_cpp::Plugin)
