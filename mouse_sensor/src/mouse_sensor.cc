#include "mouse_sensor.hh"

double MouseSensor::getDistance(int pixels) const{
    return ((double) pixels)*(INCHES_PER_METER/_dpi);
}

void MouseSensor::setRunning(bool running){
    _run_mutex.lock();
    _running = running;
    _run_mutex.unlock();
}

MouseSensor::MouseSensor(double dpi){
    _dpi = dpi;
    _display = XOpenDisplay(NULL);
    if (_display == NULL){
        fprintf(stderr, "Cannot open display\n");
        exit(-1);
    }
    _screen = DefaultScreen(_display);
    _used_screen = XScreenOfDisplay(_display, _screen);
    _root_window = RootWindow(_display, _screen);

    int pointerGrabbed = XGrabPointer(_display, _root_window, False, PointerMotionMask, GrabModeAsync, GrabModeAsync, None, None, CurrentTime);

    if(pointerGrabbed == GrabSuccess){
        fprintf(stdout, "Grabbed pointer successfully\n");
    }else{
        fprintf(stderr, "Error grabbing pointer: ");
        switch(pointerGrabbed){
            case BadCursor:
                fprintf(stderr, "Bad Cursor");
                break;
            case BadValue:
                fprintf(stderr, "Bad Value");
                break;
            case BadWindow:
                fprintf(stderr, "Bad Window");
                break;
            default:
                fprintf(stderr, "Other Error");
                break;
        }     
        fprintf(stderr, "\n");       
        exit(-2);
    }

    _center.x = XWidthOfScreen(_used_screen)/2;
    _center.y = XHeightOfScreen(_used_screen)/2;

    _position.x = 0.0;
    _position.y = 0.0;

    setRunning(true);

}

MouseSensor::~MouseSensor(){
    setRunning(false);
    XCloseDisplay(_display);
}

void MouseSensor::run(){
    XEvent event;
    int local_x, local_y;
    int x_diff, y_diff;

    _run_mutex.lock();
    while(_running){
        _run_mutex.unlock();
        XNextEvent(_display, &event);
        if(event.type == MotionNotify){
            local_x = event.xmotion.x;
            local_y = event.xmotion.y;
            XWarpPointer(_display, _root_window, _root_window, local_x-5, local_y-5, 10, 10, _center.x, _center.y);
            x_diff = local_x - _center.x;
            y_diff = local_y - _center.y;
            _position.x += getDistance(x_diff);
            _position.y += getDistance(y_diff);
            if(x_diff != 0 || y_diff != 0){
                fprintf(stdout, "X = %lf mm\n", 1000.0*_position.x);
                fprintf(stdout, "Y = %lf mm\n", 1000.0*_position.y);
            }
        }

        _run_mutex.lock();
    }

}


double MouseSensor::getX() const{
    return _position.x;
}

double MouseSensor::getY() const{
    return _position.y;
}
std::tuple<double, double> MouseSensor::getPosition() const{
    return std::make_tuple(_position.x, _position.y);
}

