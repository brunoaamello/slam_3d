#include "robot_input.hh"

void RobotInput::forceQuit(){
    exit(0);
}

void RobotInput::checkQuit(){
    if(_quit.ctrl && _quit.c){
        printf("Quit command\n");
        setRunning(false);
    }
}

void RobotInput::setRunning(bool running){
    _run_mutex.lock();
    _running = running;
    _run_mutex.unlock();
}

void RobotInput::updateVelocity(){
    if(_commands.forward && _commands.backward){
        _velocity.y = 0;
    }else if(_commands.forward){
        _velocity.y = RobotInput::VELOCITY;
    }else if(_commands.backward){
        _velocity.y = -RobotInput::VELOCITY;
    }else{
        _velocity.y = 0;
    }

    if(~(_commands.right^_commands.left)){
        _velocity.x = 0;
    }else if(_commands.right){
        _velocity.x = RobotInput::VELOCITY;
    }else{
        _velocity.x = -RobotInput::VELOCITY;
    }

    if(~(_commands.up^_commands.down)){
        _velocity.z = 0;
    }else if(_commands.up){
        _velocity.z = RobotInput::VELOCITY;
    }else{
        _velocity.z = -RobotInput::VELOCITY;
    }

    if(_commands.ang_right && _commands.ang_left){
        _velocity.ang = 0;
    }else if(_commands.ang_right){
        _velocity.ang = RobotInput::ANGULAR_VELOCITY;
    }else if(_commands.ang_left){
        _velocity.ang = -RobotInput::ANGULAR_VELOCITY;
    }else{
        _velocity.ang = 0;
    }
}

RobotInput::RobotInput(){
    _display = XOpenDisplay(NULL);
    if (_display == NULL){
        fprintf(stderr, "Cannot open display\n");
        exit(-1);
    }
    _screen = DefaultScreen(_display);
    _used_screen = XScreenOfDisplay(_display, _screen);
    _root_window = RootWindow(_display, _screen);

    int grabbedKeyboard = XGrabKeyboard(_display, _root_window, True, GrabModeAsync, GrabModeAsync, CurrentTime);

    if(grabbedKeyboard == GrabSuccess){
        fprintf(stdout, "Grabbed keys successfully\n");
    }else{
        char _error[100];
        XGetErrorText(_display, grabbedKeyboard, _error, 100);
        fprintf(stderr, "Error grabbing keys: %s\n", _error);
        exit(-2);
    }


    _velocity.x = 0.0;
    _velocity.y = 0.0;
    _velocity.z = 0.0;
    _velocity.ang = 0.0;

    _commands.ang_left = false;
    _commands.ang_right = false;
    _commands.backward = false;
    _commands.down = false;
    _commands.forward = false;
    _commands.left = false;
    _commands.right = false;
    _commands.up = false;

    _quit.ctrl = false;
    _quit.c = false;

    setRunning(true);

}

RobotInput::~RobotInput(){
    setRunning(false);
    XCloseDisplay(_display);
}

void RobotInput::run(){
    XEvent event;
    bool pressed;
    bool update_velocity;

    _run_mutex.lock();
    while(_running){
        _run_mutex.unlock();
        XNextEvent(_display, &event);
        if(event.type == KeyPress || event.type == KeyRelease){
            update_velocity = true;
            if(event.type == KeyPress){
                pressed = true;
            }else if(event.type == KeyRelease){
                pressed = false;
            }else{
                continue;
            }
            switch(event.xkey.keycode){
                case RobotInput::forward_keycode:
                    _commands.forward = pressed;
                    if(pressed){
                        printf("Pressed forward\n");
                    }
                    break;
                case RobotInput::backward_keycode:
                    _commands.backward = pressed;
                    if(pressed){
                        printf("Pressed backward\n");
                    }
                    break;
                case RobotInput::left_keycode:
                    _commands.left = pressed;
                    if(pressed){
                        printf("Pressed left\n");
                    }
                    break;
                case RobotInput::right_keycode:
                    _commands.right = pressed;
                    if(pressed){
                        printf("Pressed right\n");
                    }
                    break;
                case RobotInput::up_keycode:
                    _commands.up = pressed;
                    if(pressed){
                        printf("Pressed up\n");
                    }
                    break;
                case RobotInput::down_keycode:
                    _commands.down = pressed;
                    if(pressed){
                        printf("Pressed down\n");
                    }
                    break;
                case RobotInput::ang_left_keycode:
                    _commands.ang_left = pressed;
                    if(pressed){
                        printf("Pressed ang_left\n");
                    }
                    break;
                case RobotInput::ang_right_keycode:
                    _commands.ang_right = pressed;
                    if(pressed){
                        printf("Pressed ang_right\n");
                    }
                    break;
                case RobotInput::c_keycode:
                    _quit.c = pressed;
                    checkQuit();
                    update_velocity = false;
                    break;
                case RobotInput::l_ctrl_keycode:
                    _quit.ctrl = pressed;
                    checkQuit();
                    update_velocity = false;
                    break;

                default:
                    update_velocity = false;
                    break;
            }
        }
        //printf("Commands: {fw:%s; bw:%s; left:%s; right:%s; up:%s; down:%s; ang_left:%s; ang_right:%s}\n", _commands.forward ? "true" : "false", _commands.backward ? "true" : "false", _commands.left ? "true" : "false", _commands.right ? "true" : "false", _commands.up ? "true" : "false", _commands.down ? "true" : "false", _commands.ang_left ? "true" : "false", _commands.ang_right ? "true" : "false");
        _run_mutex.lock();
    }
    forceQuit();
    XUngrabKeyboard(_display, CurrentTime);
    _run_mutex.unlock();
}


robot_input::velocity_message RobotInput::getVelocityMsg(){
    updateVelocity();
    robot_input::velocity_message msg;
    msg.x = _velocity.x;
    msg.y = _velocity.y;
    msg.z = _velocity.z;
    msg.ang = _velocity.ang;
    //printf("Sent new message: {%lf, %lf, %lf, %lf}\n", msg.x, msg.y, msg.z, msg.ang);
    return msg;
}

