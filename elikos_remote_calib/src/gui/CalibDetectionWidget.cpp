#include "elikos_remote_calib/gui/CalibDetectionWidget.h"


namespace remote_calib{

CalibDetectionWidget::CalibDetectionWidget(QWidget* parent, const std::string& nodeName)
    : NodeCalibWidget(parent, nodeName)
{
    ui_.setupUi(NodeCalibWidget::getPanelParent());
}














}//end namespace
