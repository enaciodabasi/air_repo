#include "../include/manual_control_widget.hpp"

#include <QDebug>

ManualControlWidget::ManualControlWidget(
    std::shared_ptr<CommsHandler> comms_handler, QWidget* parent
)   : QWidget(parent)
{

    m_CommsHandler = comms_handler;

    m_Controller = new Controller();

    m_MainLayout = new QVBoxLayout();

    initControlButtons();

    m_ControlBtnsLayout = new QGridLayout();
    int r = 0;
    int c = 0;
    for(std::size_t i = 0; i < m_ControlButtons.size(); i++)
    {
        if(i == 3 || i == 6)
        {
            r += 1;
            c = 0;
        }

        m_ControlBtnsLayout->addWidget(m_ControlButtons.at(i), r, c);
        c += 1;
    }

    m_VelControlsLayout = new QHBoxLayout();

    m_LinVelIncBtn = new QPushButton("+");
    m_LinVelDecBtn = new QPushButton("-");
    m_LinVelValue = new QLineEdit();
    m_LinVelValue->setFixedSize(50, 25);
    m_LinVelValue->setText(QString::number(m_Controller->getVel(), 'f', 3));

    m_AngVelIncBtn = new QPushButton("+");
    m_AngVelDecBtn = new QPushButton("-");
    m_AngVelValue = new QLineEdit();
    m_AngVelValue->setFixedSize(50, 25);
    m_AngVelValue->setText(QString::number(m_Controller->getTurn(), 'f', 3));


    m_VelControlsLayout->addWidget(m_LinVelIncBtn);
    m_VelControlsLayout->addWidget(m_LinVelValue);
    m_VelControlsLayout->addWidget(m_LinVelDecBtn);
    
    m_VelControlsLayout->addWidget(m_AngVelIncBtn);
    m_VelControlsLayout->addWidget(m_AngVelValue);
    m_VelControlsLayout->addWidget(m_AngVelDecBtn);
    
    m_MainLayout->addLayout(m_ControlBtnsLayout);
    m_MainLayout->addLayout(m_VelControlsLayout);
    this->setLayout(m_MainLayout);

    installEventFilter(this);   

    this->setFocusPolicy(Qt::FocusPolicy::ClickFocus);

    connect(
        m_LinVelIncBtn,
        &QPushButton::clicked,
        this,
        &ManualControlWidget::onChangeVelBtns
    );

    connect(
        m_LinVelDecBtn,
        &QPushButton::clicked,
        this,
        &ManualControlWidget::onChangeVelBtns
    );

    connect(
        m_AngVelIncBtn,
        &QPushButton::clicked,
        this,
        &ManualControlWidget::onChangeTurnBtns
    );

    connect(
        m_AngVelDecBtn,
        &QPushButton::clicked,
        this,
        &ManualControlWidget::onChangeTurnBtns
    );

    qDebug() << m_Controller->getVel() << " " << m_Controller->getTurn();

}

ManualControlWidget::~ManualControlWidget()
{
    if(m_Controller)
        delete m_Controller;
}

void ManualControlWidget::onChangeVelBtns()
{
    QObject* s = sender();

    if(s == m_LinVelIncBtn)
    {
        m_Controller->increaseVel();
    }
    else if(s == m_LinVelDecBtn)
    {
        m_Controller->decreaseVel();
    }

    m_LinVelValue->setText(QString::number(m_Controller->getVel(), 'f', 3));
}

void ManualControlWidget::onChangeTurnBtns()
{
    QObject* s = sender();

    if(s == m_AngVelIncBtn)
    {
        m_Controller->increaseTurn();
    }
    else if(s == m_AngVelDecBtn)
    {
        m_Controller->decreaseTurn();
    }

    m_AngVelValue->setText(QString::number(m_Controller->getTurn(), 'f', 3));
}

void ManualControlWidget::keyPressEvent(QKeyEvent* event)
{
    
    double linX = 0.0;
    double angZ = 0.0;

    std::pair<int, int> xz;

    switch (event->key())
    {
        case Qt::Key_T:
        {
            xz = m_Controller->getKeyCoeffs("T");
            break;
        }
        case Qt::Key_Y:
        {
            xz = m_Controller->getKeyCoeffs("Y");
            break;
        }
        case Qt::Key_U:
        {
            xz = m_Controller->getKeyCoeffs("U");
            break;
        }
        case Qt::Key_G:
        {
            xz = m_Controller->getKeyCoeffs("G");
            break;
        }
        case Qt::Key_H:
            break;
        case Qt::Key_J:
        {
            xz = m_Controller->getKeyCoeffs("J");
            break;
        }
        case Qt::Key_B:
        {
            xz = m_Controller->getKeyCoeffs("B");
            break;
        }
        case Qt::Key_N:
        {
            xz = m_Controller->getKeyCoeffs("N");
            break;
        }
        case Qt::Key_M:
        {
            xz = m_Controller->getKeyCoeffs("M");
            break;
        }
        default:
            event->ignore();
    }

    if(event->isAccepted())
    {
        linX = xz.first * m_Controller->getVel();
        angZ = xz.second * m_Controller->getTurn();
        //qDebug() << linX << " " << angZ;

        m_CommsHandler->setVels(linX, angZ);
    }
    
}

void ManualControlWidget::initControlButtons()
{
    if(m_ControlButtons.size() != 9)
    {
        m_ControlButtons.resize(9);
    }

    for(std::size_t i = 0; i < m_ControlButtons.size(); i++)
    {
        if(!m_ControlButtons.at(i))
        {
            m_ControlButtons.at(i) = new QPushButton(QString::fromStdString(m_ControlKeyNames.at(i)));
        }

        //m_ControlButtons.at(i)->setEnabled(false);
    }

}

ManualControlWidget::Controller::Controller()
{
    m_KeyCoeffMap["T"] = std::make_pair(1, 1);
    m_KeyCoeffMap["Y"] = std::make_pair(1, 0);
    m_KeyCoeffMap["U"] = std::make_pair(1, -1);
    m_KeyCoeffMap["G"] = std::make_pair(0, 1);
    //m_KeyCoeffMap["H"] = std::make_pair(0, 1);
    m_KeyCoeffMap["J"] = std::make_pair(0, -1);
    m_KeyCoeffMap["B"] = std::make_pair(-1, -1);
    m_KeyCoeffMap["N"] = std::make_pair(-1, 0);
    m_KeyCoeffMap["M"] = std::make_pair(-1, 1);

    m_Vel = 0.5;
    m_Turn = 0.5;
    
}

std::pair<int, int> ManualControlWidget::Controller::getKeyCoeffs(const std::string& key)
{
    auto found = m_KeyCoeffMap.find(key);
    if(found != m_KeyCoeffMap.end())
    {
        return found->second;
    }

    return std::make_pair(0, 0);
}