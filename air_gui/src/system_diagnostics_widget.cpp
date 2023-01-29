#include "../include/system_diagnostics_widget.hpp"

#include <QDebug>
#include <stdlib.h>

SystemDiagnosticsWidget::SystemDiagnosticsWidget(QWidget* parent)
    : QWidget(parent)
{   

    m_NodeStatusMap[NodeStatus::UNKNOWN] = "Unknown";
    m_NodeStatusMap[NodeStatus::INACTIVE] = "Inactive";
    m_NodeStatusMap[NodeStatus::ACTIVE] = "Active";
    
    m_NodeNames = loadNodesToMonitor(); // Get node names from the config file
    m_NodeNamesMap = mapNodeNames(m_NodeNames); // Map the node names

    initNodeLabels();

    m_LabelsLayout = new QFormLayout();

    m_LabelsLayout->addRow(new QLabel("Monitored"), new QLabel("Status"));

    for(const auto& it : m_NodeLabelsWithIDs)
    {
        m_LabelsLayout->addRow(it.second, m_StatusLabelsWithIDs.find(it.first)->second);
    }

    this->setLayout(m_LabelsLayout);

    m_UpdateTimer = new QTimer(this);

    connect(
        m_UpdateTimer,
        &QTimer::timeout,
        this,
        QOverload<>::of(&SystemDiagnosticsWidget::update)
    );

    m_UpdateTimer->start(1000);

}

std::vector<std::string> SystemDiagnosticsWidget::loadNodesToMonitor()
{   
    std::vector<std::string> nodesToMonitor;
    
    YAML::Node config = YAML::LoadFile("/home/"+ std::string(std::getenv("USER")) + "/catkin_ws/src/air_gui/config/config.yaml");

    YAML::Node nodeNamesSeq = config["nodes_to_monitor"];
    
    nodesToMonitor = nodeNamesSeq.as<std::vector<std::string>>();

    //for(YAML::const_iterator iter = config.begin(); iter != config.end(); ++iter)
    //{
    //    nodesToMonitor.emplace_back(iter->second.as<std::string>());
    //    qDebug() << nodesToMonitor.size();
    //    qDebug() << QString::fromStdString(iter->second.as<std::string>());
    //}

    return nodesToMonitor;
} 

std::map<std::string, int> SystemDiagnosticsWidget::mapNodeNames(const std::vector<std::string>& node_names)
{
    std::map<std::string, int> tempMap;

    std::vector<std::string>::const_iterator cIter = node_names.cbegin(); // Constant iterator for the for loop.
    uint8_t nodeCount{0}; // To map the strings to integers, won't use these values later, just the string keys.

    for(cIter; cIter != node_names.cend(); cIter++)
    {
        tempMap[*cIter] = (int)nodeCount;
        nodeCount += 1;
    }

    return tempMap;

}

std::vector<std::string> SystemDiagnosticsWidget::getActiveNodes()
{   
    if(!ros::master::check()) // if master is not active, send an empty list of nodes
    {
        return std::vector<std::string>();
    }

    std::vector<std::string> tempActiveNodeNames;
    
    ros::master::getNodes(tempActiveNodeNames);

    //std::vector<std::string>::const_iterator tempNodesCIter = tempActiveNodeNames.cbegin();

    for(std::size_t i = 0; i < tempActiveNodeNames.size(); i++)
    {
        auto found = m_NodeNamesMap.find(tempActiveNodeNames[i]);
        if(found != m_NodeNamesMap.end())
            continue;
        
        tempActiveNodeNames.erase(tempActiveNodeNames.begin() + i);
    }
    
    return tempActiveNodeNames;

}

std::vector<std::string> SystemDiagnosticsWidget::findInactiveNodes(const std::vector<std::string>& current_active_nodes)
{   
    // If the size of the active node vector is equal to the map, return an empty vector.
    // This means everything is fine.
    if(current_active_nodes.size() == m_NodeNamesMap.size())
    {
        return std::vector<std::string>();
    }

    std::vector<std::string> inactiveNodes;

    for(std::size_t i = 0; i < m_NodeNames.size(); i++)
    {
        auto found = std::find(
            current_active_nodes.begin(),
            current_active_nodes.end(),
            m_NodeNames.at(i)
        );

        if(found == current_active_nodes.cend())
        {
            inactiveNodes.emplace_back(m_NodeNames.at(i));
        }
    }

    return inactiveNodes;    

} 

void SystemDiagnosticsWidget::initNodeLabels()
{   
    if(m_NodeNames.empty())
    {
        return;
    }
    std::vector<std::string>::const_iterator cIter = m_NodeNames.cbegin();
    
    m_NodeLabelsWithIDs["master"] = new QLabel("ROS Master");
    m_StatusLabelsWithIDs["master"] = new QLabel(m_NodeStatusMap.find(NodeStatus::UNKNOWN)->second);

    for(cIter; cIter != m_NodeNames.cend(); cIter++)
    {
        m_NodeLabelsWithIDs[*cIter] = new QLabel(QString::fromStdString(*cIter));
        m_StatusLabelsWithIDs[*cIter] = new QLabel(m_NodeStatusMap.find(NodeStatus::UNKNOWN)->second);
    }

}

void SystemDiagnosticsWidget::update()
{
    
    std::thread update_thread(&SystemDiagnosticsWidget::update_helper, this);

    if(update_thread.joinable())
    {
        update_thread.join();
    }
}

void SystemDiagnosticsWidget::update_helper()
{
    std::vector<std::string> activeNodes = getActiveNodes();
    if(activeNodes.empty())
    {
        m_StatusLabelsWithIDs.find("master")->second->setText(m_NodeStatusMap.find(NodeStatus::INACTIVE)->second);
        m_StatusLabelsWithIDs.find("master")->second->setPalette(QPalette(Qt::red));
        return;
    }

    m_StatusLabelsWithIDs.find("master")->second->setText(m_NodeStatusMap.find(NodeStatus::ACTIVE)->second);

    std::vector<std::string> inactiveNodes = findInactiveNodes(activeNodes);
    if(inactiveNodes.empty())
    {
        std::vector<std::string>::const_iterator cIter = m_NodeNames.cbegin();
        auto activeText = m_NodeStatusMap.find(NodeStatus::ACTIVE)->second;
        
        for(cIter; cIter != m_NodeNames.cend(); cIter++)
        {
            m_StatusLabelsWithIDs.find(*cIter)->second->setText(activeText);
            m_StatusLabelsWithIDs.find(*cIter)->second->setPalette(QPalette(Qt::green));
        }

        return;
    }

    std::vector<std::string>::const_iterator cIter = inactiveNodes.cbegin();

    const QString inactiveText = m_NodeStatusMap.find(NodeStatus::INACTIVE)->second;
    for(cIter; cIter != inactiveNodes.cend(); cIter++)
    {
        m_StatusLabelsWithIDs.find(*cIter)->second->setText(inactiveText);
        m_StatusLabelsWithIDs.find(*cIter)->second->setPalette(QPalette(Qt::red));
    }

}

