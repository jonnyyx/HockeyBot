#include "decider.h"


#define DEBUG 0

Decider::Decider(QApplication *qt_app)
{
    /* Set parameters */

    /* Initialize variable(s) */
    m_classificator = new Classification();
    m_cartographer = new Cartographer();
    m_navigator = new Navigation(m_classificator);
    m_qt_app = qt_app;
    m_team_color = UNDEFINED;
    m_field_ratio = 0.4;
    m_goal_counter = 0;
    is_area_covered = true;


    /* Initialize control variable(s) */
    m_state_current = DEFAULT;
    m_state_previous = DEFAULT;

    /* referee Initialization */
    m_referee = new Referee(8,this);
    // Slots
    connect(m_referee, SIGNAL(gameStart()), this, SLOT(slotGameStart()));
    connect(m_referee, SIGNAL(detectionStart()), this, SLOT(slotDetectionStart()));
    connect(m_referee, SIGNAL(gameOver()), this, SLOT(slotGameOver()));
    connect(m_referee, SIGNAL(abValues(double,double)), this, SLOT(slotAbValues(double,double)));
    connect(m_referee, SIGNAL(stopMovement()), this, SLOT(slotStopMovement()));
    connect(m_referee, SIGNAL(trueColorOfTeam(TeamColor)), this, SLOT(slotTeamColor(TeamColor)));
    connect(m_referee, SIGNAL(connected()), this, SLOT(slotConnected()));
    // Connection
    std::string ip_string;
    m_classificator->m_node.getParam("IP", ip_string);
    m_referee->connectToServer(QString::fromStdString(ip_string),10000);
    // m_referee->connectToServer("127.0.0.1",10000);

    /* Timer(s) */
    m_active_timer = new QTimer(this);
    connect(m_active_timer, SIGNAL(timeout()), this, SLOT(slotSendAlive()));

    is_goal_positioned = true;
}


void Decider::main()
{
    ros::Rate rate(10);

    while (ros::ok()){
        /* Run Callbacks*/
        ros::spinOnce();

        if (DEBUG){
            ROS_INFO_STREAM("Decider -- main: m_state = " << m_state_current);
        }

        /* Intercept QT Interrupt(s) */
        m_qt_app->processEvents();

        /* Game Logic */
        switch (m_state_current) {
        case DEFAULT:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = DEFAULT");
            }
            break;
        case CONNECT:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = CONNECT");
            }
            m_referee->reportReady();
            m_active_timer->start(TIMER_RATE_MS);
            updateState(INITIALIZE_READY);
            break;
        case INITIALIZE_READY:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = INITIALIZE_READY");
            }
            break;
        case INITIALIZE:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = INITIALIZE");
            }
            m_field_ratio = m_cartographer->getFieldRatio();
            m_team_color = m_classificator->getTeamColor();
            if(m_cartographer->getIsInitialized() && m_team_color != UNDEFINED){
                m_referee->tellAbRatio(m_field_ratio);
                m_referee->tellTeamColor(static_cast<TeamColor>(m_team_color - 1));
                if (DEBUG){
                    ROS_INFO_STREAM("Decider -- main: m_team_color = " << m_team_color);
                }
                updateState(GAME_READY);
            }
            break;
        case GAME_READY:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_team_color = " << m_team_color << " || m_field_ratio = " << m_field_ratio);
            }
            break;
        case LOCK_TARGET:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = LOCK_TARGET");
            }

            if (!m_navigator->getLocked()){
                if (DEBUG){
                    ROS_INFO_STREAM("Decider -- main: (LOCK_TARGET) || getLocked = false");
                }
                if (m_navigator->lockedTarget(m_team_color)){
                    m_target_object = m_navigator->getTargetObject();
                    if (DEBUG){
                        ROS_INFO_STREAM("Decider -- main: (LOCK_TARGET) || lockedTarget");
                    }
                    if(m_navigator->turnRobot(m_target_object.ang, true)){
                        updateState(MOVE_TARGET);
                    }
                } else {
                    if (DEBUG){
                        ROS_INFO_STREAM("Decider -- main: (LOCK_TARGET) = No object turn m_navigator->get_robot_position()[2]"<<m_navigator->get_robot_position()[2]);
                    }

                    if(is_area_covered){
                        if (m_navigator->get_robot_position()[2] < 35.0|| m_navigator->get_robot_position()[2] > 300.0){
                            m_navigator->turnRobot(10, true);
                        } else{
                            is_area_covered = false;
                        }
                    } else {
                        if(m_navigator->get_robot_position()[2] < 70.0 || m_navigator->get_robot_position()[2] > 330.0){
                            m_navigator->turnRobot(-10, true);
                        } else{
                            is_area_covered = true;
                        }
                    }
                    m_navigator->setLocked(false);
                }
            } else {
                m_target_object = m_navigator->getTargetObject();
                if(m_navigator->turnRobot(m_target_object.ang, true)){
                    updateState(MOVE_TARGET);
                }
            }

            break;
        case MOVE_TARGET:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = MOVE_TARGET");
            }
            if (!m_navigator->getLocked()){
                if (m_navigator->lockedTarget(m_team_color)){
                    m_target_object = m_navigator->getTargetObject();
                    if(m_navigator->moveRobot(m_target_object.dist, true)){
                        updateState(MOVE_GOAL);
                    }
                }
            } else {
                m_target_object = m_navigator->getTargetObject();
                if(m_navigator->moveRobot(m_target_object.dist, true)){
                    updateState(TURN_GOAL);
                }
            }
            break;

        case TURN_GOAL:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = TURN_GOAL");
            }

            if (is_goal_positioned){
                goal_coordinates_buffer = *m_navigator->getGoalCoordinates();
                if (DEBUG){
                    ROS_INFO_STREAM("Decider -- main: MOVE_GOAL || goal_coordinates[0] = " << goal_coordinates_buffer[0]);
                    ROS_INFO_STREAM("Decider -- main: MOVE_GOAL || goal_coordinates[1] = " << goal_coordinates_buffer[1]);
                }
                is_goal_positioned = false;
            }

            if(m_navigator->turnRobot(goal_coordinates_buffer[1], true)){
                if (DEBUG){
                    ROS_INFO_STREAM("Decider -- main: MOVE_GOAL || turnRobot = true");
                }
                updateState(MOVE_GOAL);
            }
            break;

        case MOVE_GOAL:

            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = MOVE_GOAL");
            }

            if(m_navigator->moveRobot(goal_coordinates_buffer[0], true)){
                if (DEBUG){
                    ROS_INFO_STREAM("Decider -- main: MOVE_GOAL || moveRobot = true");
                }
                is_goal_positioned = true;
                m_goal_counter += 1;
                m_referee->reportGoal();
                if (m_goal_counter < 3){
                    updateState(TURN_BACK);
                }else{
                    // Game Won
                    m_referee->reportDone();
                    updateState(GAME_OVER);
                }
            }
            break;

        case TURN_BACK:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = TURN_BACK");
            }
            if(m_navigator->turnRobot(fmod(-1.0*m_navigator->get_robot_position()[2] + 0.5, 360.0), true)){
                updateState(MOVE_BACK);
            }
            break;
        case MOVE_BACK:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = MOVE_BACK");
            }

            distance_to_drive = 2.0*m_a - (2.0/3.0*m_a - m_navigator->get_robot_position()[0]);
            if(m_navigator->moveRobot(distance_to_drive, false)){
                updateState(LOCK_TARGET);
            }
            is_area_covered = true;
            break;
        case PAUSE:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = PAUSE");
            }
            break;
        case EXIT:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = EXIT");
            }
            break;
        case GAME_OVER:
            if (DEBUG){
                ROS_INFO_STREAM("Decider -- main: m_state = GAME_OVER");
            }
            break;
        default:
            break;
        }

        rate.sleep();
    }
}
/*
 *updates state and saves previous state
*/
void Decider::updateState(State new_state){
    m_state_previous = m_state_current;
    m_state_current = new_state;
}
/*
 *slots given by angelina
*/
void Decider::slotConnected()
{
    updateState(CONNECT);
}

void Decider::slotSendAlive()
{
    m_referee->sendAlive();
}

void Decider::slotGameStart()
{
    m_navigator->setIsActive(true);

    switch(m_state_current){
    /* In case A/B or team color was not yet computed */
    case GAME_READY:
        updateState(LOCK_TARGET);
        break;
    case INITIALIZE:
        m_referee->tellAbRatio(m_field_ratio);
        m_referee->tellTeamColor(static_cast<TeamColor>(m_team_color));
        updateState(LOCK_TARGET);
        break;
    case PAUSE:
        updateState(m_state_previous);
        break;
    default:
        break;
    }
}

void Decider::slotDetectionStart()
{
    m_navigator->setIsActive(true);

    switch(m_state_current){
    case INITIALIZE_READY:
        updateState(INITIALIZE);
        ROS_INFO_STREAM("Decider -- slotDetectionStart: INITIALIZE");
        break;
    case PAUSE:
        updateState(m_state_previous);
        break;
    default:
        break;
    }
}

void Decider::slotGameOver()
{
    m_navigator->setIsActive(false);
    updateState(GAME_OVER);
}

void Decider::slotStopMovement()
{
    m_navigator->setIsActive(false);
    updateState(PAUSE);
}
void Decider:: slotAbValues(double a, double b)
{
    m_a = a;
    m_b = b;
    m_navigator->setAB(a, b);

}
void Decider::slotTeamColor(TeamColor color){
    m_classificator->setTeamColor(int(color) + 1); // Different Indices for color
    m_team_color = int(color) + 1;
}
