#include <elikos_remote_calib/model/Calib.h>
#include <iostream>
#include <iomanip>
#include <ros/this_node.h>
#include <elikos_remote_calib_client/Calibrator.h>

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
* Le constructeur de la classe Calib. Initialise tout ce dont la classe a besoin 
* pour opérer correctement avec ROS.
* @param 
*******************************************************************************/
Calib::Calib(Observer* observer)
    : observer_(observer)
{
    publisher_ = nodeHandle_.advertise<elikos_ros::CalibPreprocessing>("remote_calib_color", 2);
}

/*******************************************************************************
* Rafraichit ros, et permet la réception de messages ainsi que leurs traitement.
* Essentiellement, appelle ros::spinOnce().
*******************************************************************************/
void Calib::refresh()
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
void Calib::getCalibratableNodes(std::vector<std::string>& calibratableNodes)
{
    for(std::pair<std::string, NodeType> elem : calibratableNodes_){
        calibratableNodes.push_back(elem.first);
    }
}

/*******************************************************************************
* Cette méthode rafraichit la liste interne de noeuds calibrables. Les noeuds,
* ainsi que leurs types, sont déterminés.
*******************************************************************************/
void Calib::refreshCalibratableNodes()
{
    calibratableNodes_.clear();
    std::map<std::string, NodeType> potentialCalibratableNodes;

    std::string calibratableTopicEnd = "/" + remote_calib::REMOTE_CALIB_NAMESPCACE + "/" + remote_calib::MESSAGE_TOPIC_NAME;
    {//Rapatriment initial des topics et filtrage par type
        XmlRpc::XmlRpcValue args, result, payload;
        args[0] = ros::this_node::getName();

        bool sucess = ros::master::execute("getTopicTypes", args, result, payload, true);

        if(!sucess){
            return;
        }

        for(int i = 0; i < payload.size(); ++i){
            std::string topicName = payload[i][0];
            std::string topicType = payload[i][1];

            if(endsWith(topicName, calibratableTopicEnd)){
                std::string nodeName = topicName.substr(0, topicName.length() - calibratableTopicEnd.length());
                
                potentialCalibratableNodes[nodeName] = TOPIC_TYPE_TO_NODE_TYPE.at(topicType);
            }
        }

    }
    {//Verification du nombre de subscribers des topics
        XmlRpc::XmlRpcValue args, result, payload;
        args[0] = ros::this_node::getName();

        bool sucess = ros::master::execute("getSystemState", args, result, payload, true);

        if(!sucess){
            return;
        }

        XmlRpc::XmlRpcValue subscriberList = payload[1];
        for(int i = 0; i < subscriberList.size(); ++i){
            std::string topicName = subscriberList[i][0];

            auto it = potentialCalibratableNodes.begin();
            while(it != potentialCalibratableNodes.end()){

                if(topicName == it->first + calibratableTopicEnd){

                    if(subscriberList[i][1].size() > 0){
                        calibratableNodes_[it->first] = it->second;
                    }

                    it = potentialCalibratableNodes.erase(it);

                }else{
                    ++it;

                }
            }
        }
    }

    observer_->update();
}

/*******************************************************************************
* Retourne le type d'un noeud calibrable. le noeud doit être présent dans la 
* liste de noeuds calibrables.
* @param nodeName       [in] le nom du noeud dont on veut le type
* @return le type du noeud
* 
* @throws out_of_range si il n'y a pas de noeud 'nodeName'
*******************************************************************************/
NodeType Calib::getCalibratableNodeType(const std::string& nodeName){
    return calibratableNodes_.at(nodeName);
}


