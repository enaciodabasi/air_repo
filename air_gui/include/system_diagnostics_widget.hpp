#ifndef SYSTEM_DIAGNOSTICS_WIDGET_HPP
#define SYSTEM_DIAGNOSTICS_WIDGET_HPP

#include <QObject>
#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QLabel>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <ros/master.h>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <vector>
#include <thread>
#include <unordered_map>

class SystemDiagnosticsWidget : public QWidget
{
    public:

    SystemDiagnosticsWidget(QWidget* parent = nullptr);


    private:

    std::vector<std::string> m_NodeNames; // Contains the node names as strings

    std::map<std::string, int> m_NodeNamesMap; // Node name - int map container.

    std::unordered_map<std::string, QLabel*> m_NodeLabelsWithIDs;

    std::unordered_map<std::string, QLabel*> m_StatusLabelsWithIDs;

    QTimer* m_UpdateTimer;

    QFormLayout* m_LabelsLayout;

    enum NodeStatus{
        UNKNOWN,
        INACTIVE,
        ACTIVE
    };
    
    std::map<NodeStatus, QString> m_NodeStatusMap;

    /**
     * @brief Loads node names to check status for from a YAML file via the yaml-cpp library.
     * 
     * @return Array of strings that contains the node names to monitor.
     */
    std::vector<std::string> loadNodesToMonitor();

    /**
     * @brief Maps node names in an ordered_map.
     * @brief Makes it easier to access the node names later
     * @param node_names 
     * @return std::map<std::string, int> 
     */
    std::map<std::string, int> mapNodeNames(const std::vector<std::string>& node_names);
    
    /**
     * @brief Gets the name of the active nodes from the ros::master
     * 
     * @return A vector of strings containing the currently active nodes names. 
     */
    std::vector<std::string> getActiveNodes();

    /**
     * @brief Compares the currently active nodes with the nodes to be monitored.
     * 
     * @param current_active_nodes: vector containing the names of the active nodes taken from the ros::master 
     * @return Returns the names of the inactive nodes in a string vector. 
     */
    std::vector<std::string> findInactiveNodes(const std::vector<std::string>& current_active_nodes);


    void initNodeLabels();

    /**
     * @brief Updates the widget and the node statuses.
     * 
     */
    void update();

    void update_helper();

} ;

#endif // SYSTEM_DIAGNOSTICS_WIDGET_HPP