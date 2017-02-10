#ifndef GUI_CALIB_ELIKOS_DETECTION_H
#define GUI_CALIB_ELIKOS_DETECTION_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

namespace remote_calib{


class CalibElikosDetection
    : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    CalibElikosDetection();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings);
    virtual void restoreSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instanceSettings);

private:
    QWidget* widget_;


};




}

#endif