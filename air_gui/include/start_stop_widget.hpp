#ifndef START_STOP_WIDGET_HPP
#define START_STOP_WIDGET_HPP

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <memory>

#include "comms_handler.hpp"

class RoundButton;

class StartStopWidget : public QWidget
{
    Q_OBJECT

    public:

    StartStopWidget(std::shared_ptr<CommsHandler> comms_handler, QWidget* parent = nullptr);

    private:

    std::shared_ptr<CommsHandler> m_CommsHandler;

    QVBoxLayout* m_MainLayout;

    QHBoxLayout* m_ButtonsLayout;

    QPushButton* m_EnableDriveBtn;

    QPushButton* m_DisableDriveBtn;

    private: // Private member functions

    void onEnableDriveClicked();

    void onDisableDriveClicked();

};

class RoundButton : public QPushButton
{
    
    Q_OBJECT

    public:

    explicit RoundButton(const QString& btn_text, std::pair<QString, QString> colors, QWidget* parent = nullptr);

    public slots:

    protected:

    virtual void paintEvent(QPaintEvent* ) override;

    virtual void resizeEvent(QResizeEvent* ) override;

    private:

    QString m_PrimaryColor;
    QString m_ColorPressed;
    QString m_ButtonText;

};

#endif
