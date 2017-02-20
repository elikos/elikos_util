#include <elikos_remote_calib/Core.h>

static const std::string DETECTION_TOPIC_NAME = "detection";

namespace{
/*******************************************************************************
* Vérifie si une chaine de caractères se termine avec une chaide de caractères.
*
* @param fullString :           [in] la chaine de caractères en entier
* @param end :                  [in] la chaine de caractères de fin
*
* @return si la chaine pleine se termine avec la chaine de fin
*******************************************************************************/
bool endsWith(const std::string& fullString, const std::string& end){
    if(fullString.length() >= end.length()){
        return ( 0 == fullString.compare(fullString.length() - end.length(), end.length(), end));
    }else{
        return false;
    }
}
}

/*******************************************************************************
* Le constructeur de la classe core. Initialise tout ce dont la classe a besoin 
* pour opérer correctement avec ROS.
*******************************************************************************/
Core::Core()
{
    publisher_ = nodeHandle_.advertise<elikos_ros::CalibPreprocessing>("remote_calib_color", 2);
}

/*******************************************************************************
* Rafraichit ros, et permet la réception de messages ainsi que leurs traitement.
* Essentiellement, appelle ros::spinOnce().
*******************************************************************************/
void Core::update()
{
    ros::spinOnce();
}

/*******************************************************************************
* Retrouve la liste des noeuds ros dits 'calibrables', et revoie cette liste 
* sous forme d'un' vecteur de noms. 
* @param calibratableNodes :   [out] le vecteur auquel sera ajouté la liste de
*                                    noms
*
*******************************************************************************/
void Core::getCalibratableNodes(std::vector<std::string>& calibratableNodes)
{
    for(std::pair<NodeType, std::string> elem : calibratableNodes_){
        calibratableNodes.push_back(elem.value);
    }
}

/*******************************************************************************
* Cette méthode rafraichit la liste interne de noeuds calibrables. Les noeuds,
* ainsi que leurs types, sont déterminés.
*******************************************************************************/
void Core::refreshCalibratableNodes()
{
    calibratableNodes_.clear();

    static std::vector<std::string> nodeNames;
    nodeNames.clear();
    ros::master::getNodes(nodeNames);

    for(std::string name : nodeNames){
        if(endsWith(name, DETECTION_TOPIC_NAME)){
            calibratableNodes_.emplace_back(NodeType::DETECTION, name);
        }
    }
}

void Core::sendCalibrationData(int gaussianKernelSize, int gaussianRepetitions)
{
    elikos_ros::CalibPreprocessing msg;
    msg.gaussianKernelSize = gaussianKernelSize;
    msg.gaussianRepetitions = gaussianRepetitions;
    publisher_.publish(msg);
}