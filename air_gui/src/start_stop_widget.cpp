#include "../include/start_stop_widget.hpp"

#include <QPainter>
#include <QDebug>

StartStopWidget::StartStopWidget(
    std::shared_ptr<CommsHandler> comms_handler, 
    QWidget* parent
)   : QWidget(parent)
{
    m_CommsHandler = comms_handler;

    m_MainLayout = new QVBoxLayout(this);

    m_ButtonsLayout = new QHBoxLayout();

    m_EnableDriveBtn = new RoundButton("Start", std::make_pair<QString, QString>("green", "darkgreen"));
    m_EnableDriveBtn->setMinimumSize(100, 100);
    m_DisableDriveBtn = new RoundButton("Stop", std::make_pair<QString, QString>("red", "darkred"));
    m_DisableDriveBtn->setMinimumSize(100, 100);

    m_ButtonsLayout->addWidget(m_EnableDriveBtn);
    m_ButtonsLayout->addWidget(m_DisableDriveBtn);

    m_MainLayout->addLayout(m_ButtonsLayout);

    this->setLayout(m_MainLayout);

    connect(
        m_EnableDriveBtn,
        &QPushButton::clicked,
        this,
        &StartStopWidget::onEnableDriveClicked
    );

    connect(
        m_DisableDriveBtn,
        &QPushButton::clicked,
        this,
        &StartStopWidget::onDisableDriveClicked
    );
}

void StartStopWidget::onEnableDriveClicked()
{
    m_CommsHandler->changeDriveStatus(true);
}

void StartStopWidget::onDisableDriveClicked()
{
    m_CommsHandler->changeDriveStatus(false);
}

RoundButton::RoundButton(
    const QString& btn_text,
    std::pair<QString, QString> colors,
    QWidget* parent) : QPushButton(btn_text, parent)
{
    m_PrimaryColor = colors.first;
    m_ColorPressed = colors.second;
    m_ButtonText = btn_text;
}

void RoundButton::paintEvent(QPaintEvent* )
{
    QColor background = isDown() ? QColor(m_ColorPressed) : QColor(m_PrimaryColor);
    int diameter = qMin(height(), width());

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.translate(width() / 2, height() / 2);

    painter.setPen(QPen(QColor("black"), 1.5));
    painter.setBrush(QBrush(background));
    painter.drawEllipse(QRect(-diameter / 2, -diameter / 2, diameter, diameter));

    QPainter painter2(this);
    painter2.setPen(Qt::black);
    painter2.setFont(QFont("Arial", 12, 75, false));
    painter2.setBrush(Qt::black);

    //painter2.drawText(QPoint(this->width()/3.5, this->height() / 1.7), m_ButtonText);
    painter2.drawText(QRectF(0, 0, this->width(), this->height()), Qt::AlignCenter, m_ButtonText);
}

void RoundButton::resizeEvent(QResizeEvent *e)
{
    QPushButton::resizeEvent(e);
    int diameter = qMin(height(), width())+4 ;
    int xOff =(width() -diameter ) / 2;
    int yOff =(height() - diameter) / 2;
    setMask(QRegion(xOff,yOff, diameter, diameter,QRegion::Ellipse));
}