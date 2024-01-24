class Transition
{
public:
    double check_exceptions(double calc, double change_in_value, double start_value);

    double check_duration(double calc, double current_time, double duration, double start_time, double target_angle);

    int linear_tween(double current_time, double start_value, double change_in_value, double duration, double start_time, double target_angle);

    int ease_in_out_quad(double current_time, double start_value, double change_in_value, double duration, double start_time, double target_angle);

    int ease_in_out_cubic(double current_time, double start_value, double change_in_value, double duration, double start_time, double target_angle);

    int ease_in_out_quart(double current_time, double start_value, double change_in_value, double duration, double start_time, double target_angle);
    
    int ease_in_out_quint(double current_time, double start_value, double change_in_value, double duration, double start_time, double target_angle);
};