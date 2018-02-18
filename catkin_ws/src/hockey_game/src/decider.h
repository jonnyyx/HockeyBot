#ifndef DECIDER_H
#define DECIDER_H

#ifndef Q_MOC_RUN
#include "classification.h"
#include "navigation.h"
#include "cartographer.h"
#endif

#include "referee.h"
#include <QApplication>
#include <QObject>
#include <QTimer>
#include <QDebug>

enum State{
    DEFAULT,
    CONNECT, //1
    INITIALIZE_READY, //2
    INITIALIZE, //3
    GAME_READY, //4
    LOCK_TARGET, //5
    MOVE_TARGET, //6
    TURN_GOAL,  //7
    MOVE_GOAL, //8
    TURN_BACK, // 9
    MOVE_BACK, //10
    PAUSE, //11
    EXIT, //12
    GAME_OVER //13
};

#define TIMER_RATE_MS 30000

class Decider:QObject
{
    Q_OBJECT

public:
    Decider(QApplication* qt_app);
    void main();

private:
    void updateState(State new_state);


private:
    Classification* m_classificator;
    Cartographer* m_cartographer;
    Navigation* m_navigator;

    /* Control variable(s) */
    State m_state_current;
    State m_state_previous;

    /* QT variable(s) */
    QApplication* m_qt_app;

    /* Timer */
    QTimer* m_active_timer;

    /* Referee */
    Referee* m_referee;

    /* Game Variable(s) */
    int m_team_color;
    float m_field_ratio;

    detected_object m_target_object;

    // Goal Variable
    vector<float> goal_coordinates_buffer;
    bool is_goal_positioned;
    /*other variable(s)*/
    float m_a;
    float m_b;
    int m_goal_counter;

private: /* variables relative to moving back to goal */
    float distance_to_drive;
    bool is_area_covered;



private slots:
    void slotConnected();
    void slotSendAlive();
    void slotGameStart();
    void slotDetectionStart();
    void slotGameOver();
    void slotStopMovement();
    void slotAbValues(double, double);
    void slotTeamColor(TeamColor color);

};

#endif // DECIDER_H
