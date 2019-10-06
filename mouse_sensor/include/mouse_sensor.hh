
// External libraries
#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tuple>
#include <mutex>


class MouseSensor{
    private:
        bool _running;
        std::mutex _run_mutex;
        Display *_display;
        Window _root_window;
        int _screen;
        Screen *_used_screen;
        double _dpi;
        struct {
            int x, y;
        } _center;
        struct {
            double x, y;
        } _position;

    public:
        static constexpr double INCHES_PER_METER = 0.0254;
    
    private:
        double getDistance(int pixels) const;
        void setRunning(bool running);

    public:
        MouseSensor(double dpi=300);
        ~MouseSensor();

        void run();

        double getX() const;
        double getY() const;
        std::tuple<double, double> getPosition() const;

};
