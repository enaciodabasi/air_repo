#ifndef MANUAL_CONTROL_WIDGET_HPP
#define MANUAL_CONTROL_WIDGET_HPP

#include <ros/ros.h>

#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>

#include <vector>
#include <unordered_map>

#include "comms_handler.hpp"

class ManualControlWidget : public QWidget
{
    Q_OBJECT

    public:

    ManualControlWidget(
        std::shared_ptr<CommsHandler> comms_handler, QWidget* parent = nullptr
    );

    virtual ~ManualControlWidget();

    protected:

    //bool eventFilter(QObject* o, QEvent* event) override;

    void keyPressEvent(QKeyEvent* event) override;

    private:

    class Controller
    {
        public:

        Controller();

        inline double getVel() const
        {
            return m_Vel;
        }

        inline double getTurn() const
        {
            return m_Turn;
        }

        std::pair<int, int> getKeyCoeffs(const std::string& key);

        void why()
        {
            m_Vel *= 1;
            m_Turn *= 1;
        } 

        void increaseVel()
        {
            m_Vel = std::min(m_VelLimit, m_Vel * 1.1);
        }

        void decreaseVel()
        {
            m_Vel = std::min(m_VelLimit, m_Vel* 0.9);
        }

        void increaseTurn()
        {
            m_Turn = std::min(m_TurnLimit, m_Turn * 1.1);
        }

        void decreaseTurn()
        {
            m_Turn = std::min(m_TurnLimit, m_Turn * 0.9);
        }

        private:

        const double m_VelLimit = 1.0;

        const double m_TurnLimit = 1.0;

        double m_Vel;
        
        double m_Turn;

        std::unordered_map<std::string, std::pair<int, int>> m_KeyCoeffMap;
    };

    Controller* m_Controller;
    
    std::shared_ptr<CommsHandler> m_CommsHandler;

    std::vector<QPushButton*> m_ControlButtons;

    QVBoxLayout* m_MainLayout;

    QGridLayout* m_ControlBtnsLayout;

    QHBoxLayout* m_VelControlsLayout;

    QPushButton* m_LinVelIncBtn;
    QPushButton* m_LinVelDecBtn;

    QPushButton* m_AngVelIncBtn;
    QPushButton* m_AngVelDecBtn;

    QLineEdit* m_LinVelValue;

    QLineEdit* m_AngVelValue;
    
    private: // Controller utils:

    

    std::vector<std::string> m_ControlKeyNames = {
        "T",
        "Y",
        "U",
        "G",
        "H",
        "J",
        "B",
        "N",
        "M"
    };

    private: // Private member functions

    void initControlButtons();

    void onChangeVelBtns();

    void onChangeTurnBtns();
    

};



#endif