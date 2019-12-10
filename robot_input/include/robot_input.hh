
// Own messages
#include "robot_input/velocity_message.h"

// External libraries
#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tuple>
#include <mutex>


class RobotInput{
    private:
        bool _running;
        std::mutex _run_mutex;
        Display *_display;
        Window _root_window;
        int _screen;
        Screen *_used_screen;
        struct {
            double x, y, z, ang;
        } _velocity;

        struct {
            bool forward, backward, left, right, up, down, ang_left, ang_right;
        } _commands;

        struct {
            bool ctrl, c;
        } _quit;


    public:
        static constexpr double PI = 3.1415926536;
        static constexpr double VELOCITY = 1; /// meters/second
        static constexpr double ANGULAR_VELOCITY = PI/18.0; /// rad/second

        static const int forward_keycode = 25; // W
        static const int backward_keycode = 39; // S
        static const int left_keycode = 38; // A
        static const int right_keycode = 40; // D

        static const int up_keycode = 52; // Z
        static const int down_keycode = 53; // X

        static const int ang_left_keycode = 24; // Q
        static const int ang_right_keycode = 26; // E

        static const int l_ctrl_keycode = 37; // L_CONTROL
        static const int c_keycode = 54; // C
        

    
    private:
        void forceQuit();

        void checkQuit();

        void setRunning(bool running);

        void updateVelocity();

    public:
        RobotInput();
        ~RobotInput();

        void run();

        robot_input::velocity_message getVelocityMsg();

};
