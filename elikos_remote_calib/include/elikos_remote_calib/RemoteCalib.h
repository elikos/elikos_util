#ifndef GUI_CALIB_ELIKOS_DETECTION_H
#define GUI_CALIB_ELIKOS_DETECTION_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

#include "CalibWidget.h"


namespace remote_calib{


class RemoteCalib
    : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    RemoteCalib();
    ~RemoteCalib();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings);
    virtual void restoreSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings);

private:
    QWidget* widget_;
};




}

#endif